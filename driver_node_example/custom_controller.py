#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
import time
import struct
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, BatteryState
from tf2_ros import TransformBroadcaster
import tf_transformations

class CustomControllerNode(Node):
    def __init__(self):
        super().__init__('custom_controller')

        # --- Parameters ---
        self.declare_parameter('device_port', '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_64:E8:33:83:BA:2C-if00')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('loop_rate', 60.0)
        self.declare_parameter('telemetry_rate_hz', 60.0)
        self.declare_parameter('wheel_separation', 0.210)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('enc_counts_per_rev', 1975.0)
        self.declare_parameter('motor_acceleration', 9000.0) # In ticks/sec^2, from firmware
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('twist_topic', '/cmd_vel_out')
        self.declare_parameter('battery_reading_period', 60.0) # Read battery every 60 seconds

        # Get parameters
        self.port = self.get_parameter('device_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.telemetry_rate = self.get_parameter('telemetry_rate_hz').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.enc_cpr = self.get_parameter('enc_counts_per_rev').value
        self.motor_accel = self.get_parameter('motor_acceleration').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.battery_reading_period = self.get_parameter('battery_reading_period').value

        # --- Protocol Constants ---
        self.START_BYTE = 0x7E
        self.CMD_SET_SPEEDS = 0x01
        # self.CMD_GET_ODOMETRY = 0x02 # Obsolete
        self.CMD_ODOMETRY_DATA = 0x03
        self.CMD_GET_BATTERY = 0x04
        self.CMD_BATTERY_DATA = 0x05
        self.CMD_RESET_ENCODERS = 0x06
        self.CMD_SET_ACCEL = 0x07
        self.CMD_SET_TELEMETRY_RATE = 0x08 # New command

        # (Conversion factors, publishers, subscribers are unchanged)
        # --- Conversion Factors ---
        self.ticks_per_meter = self.enc_cpr / (2 * math.pi * self.wheel_radius)
        self.rads_per_tick = (2 * math.pi) / self.enc_cpr
        self.ticks_per_rad = self.enc_cpr / (2 * math.pi)

        # --- Publishers and Subscribers ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, self.twist_topic, self.cmd_vel_callback, 10)
        
        # --- Serial Communication ---
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f"Successfully connected to {self.port}")
            time.sleep(2)
            self.initialize_mcu()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.port}: {e}")
            rclpy.shutdown()
            return

        # (State variables are unchanged)
        self.latest_twist = Twist()
        self.last_time = self.get_clock().now()
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        self.last_battery_request_time = self.get_clock().now()

        # --- Main Control Loop ---
        loop_rate_hz = self.get_parameter('loop_rate').value
        self.timer = self.create_timer(1.0 / loop_rate_hz, self.control_loop)
        self.get_logger().info("Custom controller node has been started.")

    def send_packet(self, command_id, payload=b''):
        # (This function is unchanged)
        payload_len = 2 + len(payload)
        checksum_data = bytearray([command_id]) + payload
        checksum = sum(checksum_data) & 0xFF
        packet = bytearray([self.START_BYTE, payload_len, command_id]) + payload + bytearray([checksum])
        try:
            self.serial_conn.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def initialize_mcu(self):
        """Send initialization commands to the MCU using the new protocol."""
        self.get_logger().info("Initializing MCU...")
        # Reset encoders
        self.send_packet(self.CMD_RESET_ENCODERS)
        time.sleep(0.1)
        # Set acceleration
        accel_payload = struct.pack('<I', int(self.motor_accel))
        self.send_packet(self.CMD_SET_ACCEL, accel_payload)
        time.sleep(0.1)
        # --- ADDED: Set telemetry streaming rate ---
        self.get_logger().info(f"Setting telemetry rate to {self.telemetry_rate} Hz.")
        # '<H' is unsigned 16-bit int
        telemetry_payload = struct.pack('<H', int(self.telemetry_rate))
        self.send_packet(self.CMD_SET_TELEMETRY_RATE, telemetry_payload)
        
        self.get_logger().info("MCU Initialized.")

    def cmd_vel_callback(self, msg):
        self.latest_twist = msg

    def control_loop(self):
        # 1. --- SEND MOTOR COMMANDS ---
        linear_x = self.latest_twist.linear.x
        angular_z = self.latest_twist.angular.z
        v_right_rads = (linear_x + (angular_z * self.wheel_separation / 2.0)) / self.wheel_radius
        v_left_rads = (linear_x - (angular_z * self.wheel_separation / 2.0)) / self.wheel_radius
        v_right_ticks = int(v_right_rads * self.ticks_per_rad)
        v_left_ticks = int(v_left_rads * self.ticks_per_rad)
        speed_payload = struct.pack('<hh', v_left_ticks, v_right_ticks)
        self.send_packet(self.CMD_SET_SPEEDS, speed_payload)

        # 2. --- REQUEST OTHER TELEMETRY (infrequent data) ---
        # self.send_packet(self.CMD_GET_ODOMETRY) # <-- REMOVED
        now = self.get_clock().now()
        if (now - self.last_battery_request_time).nanoseconds / 1e9 > self.battery_reading_period:
            self.send_packet(self.CMD_GET_BATTERY)
            self.last_battery_request_time = now

        # 3. --- READ AND PROCESS INCOMING DATA ---
        self.read_and_process_serial()

    # The functions read_and_process_serial, handle_packet, process_odometry,
    # and all the publishing functions remain EXACTLY THE SAME.
    # I'm omitting them for brevity.
    # ... (paste your existing helper functions here) ...
    def read_and_process_serial(self):
        """Reads all available packets from the serial buffer and processes them."""
        while self.serial_conn.in_waiting > 0:
            if self.serial_conn.read(1) == bytes([self.START_BYTE]):
                packet_len_byte = self.serial_conn.read(1)
                if not packet_len_byte: continue
                packet_len = packet_len_byte[0]

                packet_data = self.serial_conn.read(packet_len)
                if len(packet_data) != packet_len:
                    self.get_logger().warn("Incomplete packet received.")
                    continue

                received_checksum = packet_data[-1]
                calculated_checksum = sum(packet_data[:-1]) & 0xFF
                
                if received_checksum == calculated_checksum:
                    cmd_id = packet_data[0]
                    payload = packet_data[1:-1]
                    self.handle_packet(cmd_id, payload)
                else:
                    self.get_logger().warn("Checksum mismatch, discarding packet.")

    def handle_packet(self, cmd_id, payload):
        """Dispatches packets to the correct handler based on command ID."""
        if cmd_id == self.CMD_ODOMETRY_DATA:
            try:
                odom_stamp = self.get_clock().now()
                left_ticks, right_ticks = struct.unpack('<ii', payload)
                self.process_odometry(odom_stamp, left_ticks, right_ticks)
            except struct.error:
                self.get_logger().warn("Malformed odometry packet.")
        
        elif cmd_id == self.CMD_BATTERY_DATA:
            try:
                voltage, = struct.unpack('<f', payload)
                self.process_battery_voltage(voltage)
            except struct.error:
                self.get_logger().warn("Malformed battery packet.")

    def process_battery_voltage(self, voltage):
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = voltage
        battery_msg.present = True
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery_msg.current = float('nan')
        battery_msg.charge = float('nan')
        battery_msg.percentage = float('nan')
        self.battery_pub.publish(battery_msg)
    
    def process_odometry(self, current_time, left_ticks, right_ticks):
        dt_duration = current_time - self.last_time
        dt = dt_duration.nanoseconds / 1e9

        # Avoid division by zero and handle the first run
        if dt == 0:
            return

        # Handle the very first reading
        if self.last_left_ticks is None or self.last_right_ticks is None:
                self.last_left_ticks = left_ticks
                self.last_right_ticks = right_ticks
                self.last_time = current_time
                return

        # --- FORWARD KINEMATICS ---
        delta_left_ticks = left_ticks - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks

        dist_left = delta_left_ticks / self.ticks_per_meter
        dist_right = delta_right_ticks / self.ticks_per_meter

        delta_dist = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_separation

        # Update pose
        self.x_pos += delta_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y_pos += delta_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) # Normalize angle

        # Calculate velocities
        linear_velocity = delta_dist / dt
        angular_velocity = delta_theta / dt

        # --- PUBLISH ALL MESSAGES ---
        # Now call the complete helper functions
        self.publish_odometry(current_time, linear_velocity, angular_velocity)
        self.publish_tf(current_time)
        self.publish_joint_states(current_time, left_ticks, right_ticks, delta_left_ticks, delta_right_ticks, dt)

        # Update state for the next loop
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

    def publish_odometry(self, stamp, linear_vel, angular_vel):
        """
        Publishes the odometry message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set the position
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation (from yaw angle theta)
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set the velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Set covariance (these are example values, tune for your robot)
        # Tuanable: Lower values mean higher confidence
        odom_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.2] # Higher angular covariance

        odom_msg.twist.covariance = odom_msg.pose.covariance # Can be reused or set independently

        self.odom_pub.publish(odom_msg)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = self.y_pos
        t.transform.translation.z = 0.0
        
        # Set the rotation
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, stamp, total_left_ticks, total_right_ticks, delta_left, delta_right, dt):
        """
        Publishes the state of the wheel joints.
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = stamp.to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Position is the total accumulated rotation in radians
        joint_state_msg.position = [
            total_left_ticks * self.rads_per_tick,
            total_right_ticks * self.rads_per_tick
        ]
        
        # Velocity is the current wheel velocity in rad/s
        if dt > 0:
            left_velocity = (delta_left * self.rads_per_tick) / dt
            right_velocity = (delta_right * self.rads_per_tick) / dt
        else:
            left_velocity = 0.0
            right_velocity = 0.0

        joint_state_msg.velocity = [left_velocity, right_velocity]
        
        self.joint_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CustomControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        node.get_logger().info("Stopping robot...")
        stop_payload = struct.pack('<hh', 0, 0)
        node.send_packet(node.CMD_SET_SPEEDS, stop_payload)
        node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
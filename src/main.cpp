#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include "DCMotorController.h" // Your existing motor controller header
#include "protocol.h"          // Our new protocol header
#include <driver/adc.h>

//motor control variables
float tgt_lm_speed = 0, tgt_rm_speed = 0;
float curr_lm_speed = 0, curr_rm_speed = 0; 
float global_acceleration = 9000; //pulses/second^2
unsigned long lastMotorCommand = millis();
#define SILENCE_TIMEOUT 2000

// --- TELEMETRY STREAMING ---
unsigned long telemetry_period_ms = 0; // Period in ms for streaming. 0 = off.
unsigned long last_telemetry_send_ms = 0;

// --- BATTERY SAMPLING LOGIC (RESTORED) ---
#define BATTERY_ADC_SAMPLES 16 // Number of samples for the rolling average
#define BATTERY_SAMPLE_INTERVAL_MS 500 // Take a new ADC reading every 100ms
#define BATTERY_CALC_INTERVAL_MS 1000  // Recalculate the average voltage every second

int battery_readings[BATTERY_ADC_SAMPLES];
char current_read_index = 0;
float battery_voltage = 0.0f; // This global variable holds the latest filtered voltage
unsigned long last_battery_sample_ms = 0;
unsigned long last_battery_calc_ms = 0;

// --- BATTERY HELPER FUNCTIONS (RESTORED) ---
void setup_battery_readings(){
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);
  // Pre-fill the buffer for an initial reading
  for(int i = 0; i < BATTERY_ADC_SAMPLES; i++){
    battery_readings[i] = adc1_get_raw(ADC1_CHANNEL_3);
    delay(2); // Small delay between initial reads
  } 
  current_read_index = 0;
}

// Takes one new sample and adds it to the circular buffer
void sample_battery(){
  battery_readings[current_read_index] = adc1_get_raw(ADC1_CHANNEL_3);
  current_read_index = (current_read_index + 1) % BATTERY_ADC_SAMPLES;
}

// Calculates the average voltage from the buffer and updates the global variable
void calculate_battery_voltage(){
  long sum = 0;
  for(int i = 0; i < BATTERY_ADC_SAMPLES; i++){
    sum += battery_readings[i];
  }
  float average_adc = (float)sum / (float)BATTERY_ADC_SAMPLES;
  // Update the global variable. Adjust multipliers for your specific voltage divider.
  battery_voltage = (average_adc / 4095.0) * 3.3 * 7.0; 
}

// --- PROTOCOL FUNCTIONS ---
// (send_packet, calculate_checksum, send_odometry, send_battery_voltage are unchanged)
uint8_t calculate_checksum(const uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum += data[i];
    }
    return checksum;
}

void send_packet(uint8_t cmd_id, const uint8_t* payload, uint8_t payload_size) {
    uint8_t packet_len = 2 + payload_size; // cmd_id + payload + checksum
    uint8_t checksum_data[1 + payload_size];
    
    checksum_data[0] = cmd_id;
    if (payload_size > 0) {
        memcpy(&checksum_data[1], payload, payload_size);
    }
    
    uint8_t checksum = calculate_checksum(checksum_data, 1 + payload_size);

    Serial.write(START_BYTE);
    Serial.write(packet_len);
    Serial.write(cmd_id);
    if (payload_size > 0) {
        Serial.write(payload, payload_size);
    }
    Serial.write(checksum);
}

void send_odometry() {
    OdometryData odom;
    odom.left_ticks = (int32_t)leftMotorPosition;
    odom.right_ticks = (int32_t)rightMotorPosition;
    send_packet(CMD_ODOMETRY_DATA, (uint8_t*)&odom, sizeof(odom));
}

void send_battery_voltage() {
    send_packet(CMD_BATTERY_DATA, (uint8_t*)&battery_voltage, sizeof(battery_voltage));
}


void run_command(uint8_t cmd_id, const uint8_t* payload, uint8_t len) {
    switch(cmd_id) {
        case CMD_SET_SPEEDS: {
            if (len == sizeof(SpeedCommand)) {
                lastMotorCommand = millis();
                const SpeedCommand* cmd = (const SpeedCommand*)payload;
                tgt_lm_speed = (float)cmd->left_speed;
                tgt_rm_speed = (float)cmd->right_speed;
            }
            break;
        }
        // case CMD_GET_ODOMETRY: // <-- REMOVED
        case CMD_GET_BATTERY: {
            send_battery_voltage();
            break;
        }
        case CMD_RESET_ENCODERS: {
            rightMotor_encoder.setCount(0);
            leftMotor_encoder.setCount(0);
            resetPID();
            break;
        }
        case CMD_SET_ACCEL: {
            if (len == sizeof(AccelCommand)) {
                const AccelCommand* cmd = (const AccelCommand*)payload;
                global_acceleration = (float)cmd->acceleration;
            }
            break;
        }
        // --- ADDED: Handle telemetry rate setting ---
        case CMD_SET_TELEMETRY_RATE: {
            if (len == sizeof(TelemetryRateCommand)) {
                const TelemetryRateCommand* cmd = (const TelemetryRateCommand*)payload;
                if (cmd->frequency_hz > 0) {
                    telemetry_period_ms = 1000 / cmd->frequency_hz;
                } else {
                    telemetry_period_ms = 0; // 0 Hz disables streaming
                }
            }
            break;
        }
    }
}

// --- MAIN LOGIC ---
// (setup, speedLoop, and process_serial are unchanged)
void setup(){
  Serial.begin(230400);
  setupMotors();

  // Initialize the battery reading system
  setup_battery_readings();
  calculate_battery_voltage(); // Get a valid first reading

  rightMotorTargetPosition = (float)rightMotor_encoder.getCount();
  leftMotorTargetPosition = (float)leftMotor_encoder.getCount();
}

// responsible for integrating the set velocity into the target position for each motor
void speedLoop(){
  float dt = ((float) (cur_micro - last_micros))/1e6; //in seconds
  float max_accel = global_acceleration*dt;
  float lm_speed_diff = curr_lm_speed - tgt_lm_speed;
  if(abs(lm_speed_diff) <= max_accel) curr_lm_speed = tgt_lm_speed;
  else if (lm_speed_diff > max_accel) curr_lm_speed-=max_accel;
  else curr_lm_speed+=max_accel;

  float rm_speed_diff = curr_rm_speed - tgt_rm_speed;
  if(abs(rm_speed_diff) <= max_accel) curr_rm_speed = tgt_rm_speed;
  else if (rm_speed_diff > max_accel) curr_rm_speed-=max_accel;
  else curr_rm_speed+=max_accel;

  if(abs(rightMotorTargetPosition - rightMotorPosition) < 200) rightMotorTargetPosition = rightMotorTargetPosition + curr_rm_speed*dt;
  if(abs(leftMotorTargetPosition - leftMotorPosition) < 200) leftMotorTargetPosition = leftMotorTargetPosition + curr_lm_speed*dt;
  last_micros = cur_micro;
}

enum ParseState { WAIT_FOR_START, WAIT_FOR_LEN, READ_PACKET };
ParseState state = WAIT_FOR_START;
uint8_t packet_buffer[128];
uint8_t packet_len = 0;
uint8_t bytes_read = 0;

void process_serial() {
    while (Serial.available()) {
        uint8_t byte_in = Serial.read();
        switch (state) {
            case WAIT_FOR_START:
                if (byte_in == START_BYTE) { state = WAIT_FOR_LEN; }
                break;
            case WAIT_FOR_LEN:
                packet_len = byte_in;
                if (packet_len > 0 && packet_len < sizeof(packet_buffer)) {
                    bytes_read = 0;
                    state = READ_PACKET;
                } else { state = WAIT_FOR_START; }
                break;
            case READ_PACKET:
                packet_buffer[bytes_read++] = byte_in;
                if (bytes_read == packet_len) {
                    uint8_t cmd_id = packet_buffer[0];
                    const uint8_t* payload = &packet_buffer[1];
                    uint8_t payload_len = packet_len - 2;
                    uint8_t received_checksum = packet_buffer[packet_len - 1];
                    uint8_t calculated_checksum = calculate_checksum(packet_buffer, packet_len - 1);
                    if (calculated_checksum == received_checksum) {
                        run_command(cmd_id, payload, payload_len);
                    }
                    state = WAIT_FOR_START;
                }
                break;
        }
    }
}

void loop(){
  cur_micro = micros();
  unsigned long current_ms = millis();

  // Auto-stop if no command received
  if ((current_ms - lastMotorCommand) > SILENCE_TIMEOUT) {
    tgt_lm_speed = 0;
    tgt_rm_speed = 0;
  }
  
  // Handle incoming commands from ROS
  process_serial();

  // Stream odometry at the configured rate
  if (telemetry_period_ms > 0 && (current_ms - last_telemetry_send_ms >= telemetry_period_ms)) {
      send_odometry();
      last_telemetry_send_ms = current_ms;
  }

  // --- BACKGROUND BATTERY MONITORING (RESTORED) ---
  // Take a new sample at a high frequency
  if (current_ms - last_battery_sample_ms >= BATTERY_SAMPLE_INTERVAL_MS) {
      sample_battery();
      last_battery_sample_ms = current_ms;
  }
  // Recalculate the filtered average at a lower frequency
  if (current_ms - last_battery_calc_ms >= BATTERY_CALC_INTERVAL_MS) {
      calculate_battery_voltage();
      last_battery_calc_ms = current_ms;
  }

  // Run motor control loops
  if(last_micros-cur_micro > pidSampleTime) speedLoop();
  motorsLoop();
}
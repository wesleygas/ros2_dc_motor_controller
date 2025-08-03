// protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

const uint8_t START_BYTE = 0x7E;

// Commands from ROS to ESP32
const uint8_t CMD_SET_SPEEDS = 0x01;
// const uint8_t CMD_GET_ODOMETRY = 0x02; // Obsolete
const uint8_t CMD_GET_BATTERY = 0x04;
const uint8_t CMD_RESET_ENCODERS = 0x06;
const uint8_t CMD_SET_ACCEL = 0x07;
const uint8_t CMD_SET_TELEMETRY_RATE = 0x08; // New command to set streaming rate

// Commands from ESP32 to ROS
const uint8_t CMD_ODOMETRY_DATA = 0x03;
const uint8_t CMD_BATTERY_DATA = 0x05;
const uint8_t CMD_LOG_MESSAGE = 0xFF;

// --- PACKET STRUCTURES ---
struct __attribute__((packed)) SpeedCommand {
    int16_t left_speed;
    int16_t right_speed;
};

struct __attribute__((packed)) OdometryData {
    int32_t left_ticks;
    int32_t right_ticks;
};

struct __attribute__((packed)) AccelCommand {
    uint32_t acceleration;
};

// ADDED: Payload for setting the telemetry rate
struct __attribute__((packed)) TelemetryRateCommand {
    uint16_t frequency_hz;
};

#endif
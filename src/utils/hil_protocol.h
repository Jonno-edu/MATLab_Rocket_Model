#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>

/*
 * HIL (Hardware-in-the-Loop) Protocol Definition
 * 
 * Simple binary protocol for communication between Simulink and Pico W.
 * Communication is via USB serial at 115200 baud.
 * 
 * No framing bytes or checksums - just raw binary floats for maximum speed.
 */

// Input packet: Simulink -> Pico (11 floats = 44 bytes)
typedef struct {
    float current_time;     // Simulation time
    float qw, qx, qy, qz;   // Quaternion
    float p, q, r;          // Body rates
    float ref_pitch;        // Reference pitch angle
    float ref_yaw;          // Reference yaw angle
    float roll_cmd;         // Roll angle command
} __attribute__((packed)) HIL_InputPacket;

// Output packet: Pico -> Simulink (3 floats = 12 bytes)
typedef struct {
    float Y_nozzle;         // Pitch nozzle command
    float Z_nozzle;         // Yaw nozzle command
    float X_roll;           // Roll command
} __attribute__((packed)) HIL_OutputPacket;

#define HIL_INPUT_SIZE   (sizeof(HIL_InputPacket))   // 44 bytes
#define HIL_OUTPUT_SIZE  (sizeof(HIL_OutputPacket))  // 12 bytes

#endif /* HIL_PROTOCOL_H */

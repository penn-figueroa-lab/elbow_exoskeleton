// CubeMars AK45-36 MIT Mode Control with Portenta H7 on CAN1


// Libraries Required
#include <mbed.h>
#include "Arduino_PowerManagement.h"

// CAN Bus Setup (pins and rate)
mbed::CAN can1(PB_8, PH_13, 1000000);

// LED Pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Motor Configuration (set from cubemars app)
#define MOTOR_ID 2
const unsigned long CAN_ID = MOTOR_ID;

// Value Limits (from datasheet for AK series)
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -65.0f
#define T_MAX 65.0f

// Set Value Params
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 5.0f;
float kd_in = 1.0f;
float t_in = 0.0f;

// Measured Value Params
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

// Timing
uint32_t lastSendTime    = 0;
uint32_t lastReceiveTime = 0;

// ---------------------------------------------------------------
// Outer PD Loop Gains
// These act on top of the motor's internal MIT PD loop.
// Tune OUTER_KP and OUTER_KD independently from kp_in / kd_in.
// OUTER_KP: how aggressively to correct position error (rad -> rad correction)
// OUTER_KD: how aggressively to damp velocity error (rad/s -> rad correction)
// ---------------------------------------------------------------
#define OUTER_KP 1.5f   // proportional gain on position error  (tune this)
#define OUTER_KD 0.05f  // derivative gain on velocity error     (tune this)


/*********************************************************************************************************
  CONVERSION FUNCTIONS
*********************************************************************************************************/


unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int)((x - offset) * 4095.0 / span);
  }
  if (bits == 16) {
    pgg = (unsigned int)((x - offset) * 65535.0 / span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }
  return pgg;
}


/*********************************************************************************************************
  CAN COMMUNICATION SECTION 
*********************************************************************************************************/


void unpack_reply(uint8_t* dat, uint8_t len) {
  if (len != 6) return;

  unsigned int id    = dat[0];
  unsigned int p_int = (dat[1] << 8) | dat[2];
  unsigned int v_int = (dat[3] << 4) | (dat[4] >> 4);
  unsigned int i_int = ((dat[4] & 0xF) << 8) | dat[5];

  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);

  lastReceiveTime = millis();

  digitalWrite(LED_BLUE, LOW);
  delayMicroseconds(100);
  digitalWrite(LED_BLUE, HIGH);
}

void pack_cmd() {
  byte buf[8];

  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp    = constrain(kp_in, KP_MIN, KP_MAX);
  float kd    = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff  = constrain(t_in, T_MIN, T_MAX);

  unsigned int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;

  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  memcpy(msg.data, buf, 8);
  can1.write(msg);
}


/*********************************************************************************************************
  MOTOR MODE SECTION 
*********************************************************************************************************/


bool EnterMotorMode() {
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF;
  msg.data[1] = 0xFF;
  msg.data[2] = 0xFF;
  msg.data[3] = 0xFF;
  msg.data[4] = 0xFF;
  msg.data[5] = 0xFF;
  msg.data[6] = 0xFF;
  msg.data[7] = 0xFC;

  if (!can1.write(msg)) {
    return false;
  }

  for (int i = 0; i < 5; i++) {
    pack_cmd();
    delay(10);

    mbed::CANMessage msgIn;
    if (can1.read(msgIn, 0)) {
      if (msgIn.id == CAN_ID && msgIn.len == 6) {
        unpack_reply(msgIn.data, msgIn.len);
        digitalWrite(LED_GREEN, LOW);
        return true;
      }
    }
  }
  return false;
}

void Zero() {
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF;
  msg.data[1] = 0xFF;
  msg.data[2] = 0xFF;
  msg.data[3] = 0xFF;
  msg.data[4] = 0xFF;
  msg.data[5] = 0xFF;
  msg.data[6] = 0xFF;
  msg.data[7] = 0xFE;

  can1.write(msg);

  for (int i = 0; i < 5; i++) {
    delay(10);
    pack_cmd();
  }
}


// #############

// ---------------------------------------------------------------
// moveTo() — now with outer PD correction
//
// What changed vs original:
//   1. After sending the command and reading the reply, we compute
//      a position error (target - p_out) and velocity error (0 - v_out).
//   2. Those errors are fed through OUTER_KP / OUTER_KD to produce a
//      small corrective offset that is ADDED to p_in before the second
//      pack_cmd() call.
//   3. p_in is restored to the clean target after the correction send
//      so the next moveTo() call starts from the true desired position.
//   4. The dwell timing is preserved identically to the original.
// ---------------------------------------------------------------
void moveTo(float target_pos, uint32_t dwell_ms) {
  p_in = target_pos;

  // --- original: send command, wait briefly, then read the reply ---
  pack_cmd();
  delay(5);

  mbed::CANMessage msgIn;
  if (can1.read(msgIn, 0)) {
    if (msgIn.id == CAN_ID && msgIn.len == 6) {
      unpack_reply(msgIn.data, msgIn.len);
    }
  }

  // --- NEW: outer PD correction using the feedback just received ---
  float pos_error = target_pos - p_out;   // how far off we are in position
  float vel_error = 0.0f - v_out;         // we want zero velocity at setpoint

  float correction = (OUTER_KP * pos_error) + (OUTER_KD * vel_error);

  // Clamp correction to a safe window so we never command a wild jump
  correction = constrain(correction, -0.5f, 0.5f);

  // Apply correction: shift the commanded position toward where we need to be
  p_in = constrain(target_pos + correction, P_MIN, P_MAX);

  // Send the corrected command
  pack_cmd();

  // Restore p_in to the clean target for the next call
  p_in = target_pos;
  // ----------------------------------------------------------------

  // Wait the rest of the dwell time at this position
  delay(dwell_ms - 10);
}

/*********************************************************************************************************
  MAIN FUNCTION SECTION
*********************************************************************************************************/

Board board;


// #####################


void setup() {

  // For 3.3V pin outputs
  board.begin();
  board.setExternalVoltage(3.3);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE,  HIGH);

  can1.mode(mbed::CAN::Mode::Normal);
  delay(1000);

  // Zero, then enable ONCE
  p_in = 0.0f;
  Zero();
  // Drive to physical zero before starting
  //delay(1000);
  EnterMotorMode();
  //moveTo(p_in, 1000);  // go to calibrated zero, dwell 1 second
  delay(3000);

  lastSendTime    = millis();
  lastReceiveTime = millis();

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);
  digitalWrite(LED_RED,   LOW);
  delay(2000);
}



// ############################
// MAIN LOOP


void loop() {

  // // Sweep 0 → -1.57 rad (0° → 90°)
  // for (float pos = 0.0f; pos >= -1.570f; pos -= 0.005f) {
  //   p_in  = pos;
  //   v_in  = 0.0f;
  //   kp_in = 4.0f;
  //   kd_in = 2.0f;
  //   t_in  = 0.0f;
  //   pack_cmd();
  //   delay(5);
  //   mbed::CANMessage msgIn;
  //   if (can1.read(msgIn, 0))
  //     if (msgIn.id == CAN_ID && msgIn.len == 6)
  //       unpack_reply(msgIn.data, msgIn.len);
  //   delay(15);
  // }

  // // Sweep -1.57 → 0 rad (90° → 0°)
  // for (float pos = -1.570f; pos <= 0.0f; pos += 0.005f) {
  //   p_in  = pos;
  //   v_in  = 0.0f;
  //   kp_in = 4.0f;
  //   kd_in = 2.0f;
  //   t_in  = 0.0f;
  //   pack_cmd();
  //   delay(5);
  //   mbed::CANMessage msgIn;
  //   if (can1.read(msgIn, 0))
  //     if (msgIn.id == CAN_ID && msgIn.len == 6)
  //       unpack_reply(msgIn.data, msgIn.len);
  //   delay(15);
  // }


  // Sweep forward 0.0 -> -1.570 rad (reversed direction)
  for (float pos = 0.0f; pos >= -1.570f; pos -= 0.005f) {
    moveTo(pos, 20);
  }

  // Sweep back -1.570 -> 0.0 rad
  for (float pos = -1.570f; pos <= 0.0f; pos += 0.005f) {
    moveTo(pos, 20);
  }
  
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

// CubeMars AK45-36 MIT Mode Control with Portenta H7 on CAN1
// Added: Impedance Control mode alongside position sweep

// Libraries Required
#include <mbed.h>
#include "Arduino_PowerManagement.h"

// CAN Bus Setup (pins and rate)
mbed::CAN can1(PB_8, PH_13, 1000000);

// LED Pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Motor Configuration
#define MOTOR_ID 2
const unsigned long CAN_ID = MOTOR_ID;

// Value Limits (from datasheet §5.3 AK10-9 column)
#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -50.0f
#define V_MAX   50.0f
#define KP_MIN   0.0f
#define KP_MAX 500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN  -65.0f
#define T_MAX   65.0f

// Set Value Params
float p_in  = 0.0f;
float v_in  = 0.0f;
float kp_in = 5.0f;
float kd_in = 1.0f;
float t_in  = 0.0f;

// Measured Value Params
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

// Timing
uint32_t lastSendTime    = 0;
uint32_t lastReceiveTime = 0;

// Outer PD Loop Gains (software layer on top of MIT internal loop)
#define OUTER_KP 1.5f
#define OUTER_KD 0.05f

// ---------------------------------------------------------------
// IMPEDANCE CONTROL PARAMETERS
// The MIT mode equation on the driver board is:
//   τ_output = Kp × (p_des − p_actual) + Kd × (0 − v_actual) + t_ff
//
// For an elbow exoskeleton:
//   IMP_KP  : virtual spring stiffness (N·m/rad) — how hard it pulls back
//   IMP_KD  : virtual damping (N·m·s/rad)        — how much it resists motion
//   IMP_T_FF: feedforward torque (N·m)            — gravity comp or assist bias
//   IMP_SETPOINT: the equilibrium angle the spring is anchored to (rad)
//
// Tuning guide:
//   Start: KP=10, KD=1, T_FF=0  → soft compliant hold
//   Stiffer brace feel: raise KP toward 50-100
//   Reduce oscillation: raise KD toward 2-3
//   Gravity compensation: set T_FF to offset arm weight (~+1 to +3 Nm)
// ---------------------------------------------------------------
#define IMP_KP       10.0f   // virtual spring  (N·m/rad)  — START SMALL
#define IMP_KD        1.0f   // virtual damping (N·m·s/rad)
#define IMP_T_FF      0.0f   // feedforward torque (N·m), 0 = no gravity comp
#define IMP_SETPOINT  0.0f   // equilibrium angle (rad) — arm rests here

// Control mode selector — change this to switch behaviour
// 0 = position sweep (original)
// 1 = impedance control (new)
#define CONTROL_MODE  1


/*********************************************************************************************************
  CONVERSION FUNCTIONS
*********************************************************************************************************/

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) pgg = (unsigned int)((x - offset) * 4095.0  / span);
  if (bits == 16) pgg = (unsigned int)((x - offset) * 65535.0 / span);
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span   = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) pgg = ((float)x_int) * span / 4095.0  + offset;
  if (bits == 16) pgg = ((float)x_int) * span / 65535.0 + offset;
  return pgg;
}


/*********************************************************************************************************
  CAN COMMUNICATION
*********************************************************************************************************/

void unpack_reply(uint8_t* dat, uint8_t len) {
  if (len != 6) return;

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

  float p_des = constrain(p_in,  P_MIN,  P_MAX);
  float v_des = constrain(v_in,  V_MIN,  V_MAX);
  float kp    = constrain(kp_in, KP_MIN, KP_MAX);
  float kd    = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff  = constrain(t_in,  T_MIN,  T_MAX);

  unsigned int p_int  = float_to_uint(p_des, P_MIN,  P_MAX,  16);
  unsigned int v_int  = float_to_uint(v_des, V_MIN,  V_MAX,  12);
  unsigned int kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
  unsigned int t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

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
  MOTOR MODE
*********************************************************************************************************/

bool EnterMotorMode() {
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF; msg.data[1] = 0xFF; msg.data[2] = 0xFF; msg.data[3] = 0xFF;
  msg.data[4] = 0xFF; msg.data[5] = 0xFF; msg.data[6] = 0xFF; msg.data[7] = 0xFC;

  if (!can1.write(msg)) return false;

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
  msg.data[0] = 0xFF; msg.data[1] = 0xFF; msg.data[2] = 0xFF; msg.data[3] = 0xFF;
  msg.data[4] = 0xFF; msg.data[5] = 0xFF; msg.data[6] = 0xFF; msg.data[7] = 0xFE;

  can1.write(msg);
  for (int i = 0; i < 5; i++) { delay(10); pack_cmd(); }
}


/*********************************************************************************************************
  POSITION SWEEP — original moveTo with outer PD correction
*********************************************************************************************************/

void moveTo(float target_pos, uint32_t dwell_ms) {
  p_in = target_pos;

  pack_cmd();
  delay(5);

  mbed::CANMessage msgIn;
  if (can1.read(msgIn, 0)) {
    if (msgIn.id == CAN_ID && msgIn.len == 6)
      unpack_reply(msgIn.data, msgIn.len);
  }

  // Outer PD correction
  float correction = constrain(
    (OUTER_KP * (target_pos - p_out)) + (OUTER_KD * (0.0f - v_out)),
    -0.5f, 0.5f
  );

  p_in = constrain(target_pos + correction, P_MIN, P_MAX);
  pack_cmd();
  p_in = target_pos;

  delay(dwell_ms - 10);
}


/*********************************************************************************************************
  IMPEDANCE CONTROL — single function call, runs continuously in loop()
  
  How it works:
    The MIT driver board internally computes:
      τ = Kp × (p_des − p_actual) + Kd × (0 − v_actual) + t_ff
    
    We simply set:
      p_in  = IMP_SETPOINT   → the equilibrium position (virtual spring anchor)
      v_in  = 0              → we want zero velocity at equilibrium
      kp_in = IMP_KP         → virtual spring stiffness
      kd_in = IMP_KD         → virtual damping
      t_in  = IMP_T_FF       → feedforward (gravity comp / assist bias)
    
    The board then outputs torque that scales with how far/fast
    the user displaces the arm — true impedance behaviour.
    No outer PD loop needed here; we want the motor to be compliant.

  What you feel at the elbow:
    - Arm at IMP_SETPOINT    → zero torque, free
    - Arm pushed away        → spring pulls it back (proportional to KP)
    - Arm moving fast        → damping resists motion (proportional to KD)
    - Release arm            → returns to setpoint and settles (no oscillation if KD high enough)
*********************************************************************************************************/

void runImpedanceControl() {
  // Load impedance params into the global command variables
  p_in  = IMP_SETPOINT;   // virtual spring anchor point
  v_in  = 0.0f;           // desired velocity at equilibrium
  kp_in = IMP_KP;         // virtual stiffness
  kd_in = IMP_KD;         // virtual damping
  t_in  = IMP_T_FF;       // feedforward torque

  // Send command
  pack_cmd();
  delay(5);

  // Read feedback
  mbed::CANMessage msgIn;
  if (can1.read(msgIn, 0)) {
    if (msgIn.id == CAN_ID && msgIn.len == 6)
      unpack_reply(msgIn.data, msgIn.len);
  }

  // Small delay to set ~200 Hz update rate
  // Lower = more responsive, higher = smoother but laggier
  delay(5);
}


/*********************************************************************************************************
  SETUP
*********************************************************************************************************/

Board board;

void setup() {
  board.begin();
  board.setExternalVoltage(3.3);

  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   HIGH);
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  HIGH);

  can1.mode(mbed::CAN::Mode::Normal);
  delay(1000);

  p_in = 0.0f;
  Zero();
  EnterMotorMode();
  delay(3000);

  lastSendTime    = millis();
  lastReceiveTime = millis();

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);
  digitalWrite(LED_RED,   LOW);
  delay(2000);
}


/*********************************************************************************************************
  MAIN LOOP — select mode via CONTROL_MODE define at top of file
*********************************************************************************************************/

void loop() {

#if CONTROL_MODE == 0
  // ── ORIGINAL POSITION SWEEP ──────────────────────────────────────────
  for (float pos = 0.0f; pos >= -1.570f; pos -= 0.005f) moveTo(pos, 20);
  for (float pos = -1.570f; pos <= 0.0f; pos += 0.005f) moveTo(pos, 20);

#elif CONTROL_MODE == 1
  // ── IMPEDANCE CONTROL ────────────────────────────────────────────────
  // Runs continuously — arm floats at IMP_SETPOINT with virtual spring+damper.
  // Change IMP_SETPOINT, IMP_KP, IMP_KD at the top of the file to tune.
  runImpedanceControl();

#endif

}

/*********************************************************************************************************
  END FILE
  
  QUICK TUNING REFERENCE for elbow exoskeleton:
  
  Goal                          | IMP_KP  | IMP_KD | IMP_T_FF
  ------------------------------|---------|--------|----------
  Soft gravity compensation     |   5     |  0.5   |  +1.5
  Compliant assistance          |  10     |  1.0   |   0.0
  Moderate stiffness / brace    |  30     |  2.0   |   0.0
  Stiff rehabilitation hold     |  80     |  3.0   |   0.0
  
  IMP_SETPOINT: set to the desired resting elbow angle in radians
    0.0  = fully extended (straight arm)
    0.78 = 45 degrees flexed
    1.57 = 90 degrees flexed
*********************************************************************************************************/

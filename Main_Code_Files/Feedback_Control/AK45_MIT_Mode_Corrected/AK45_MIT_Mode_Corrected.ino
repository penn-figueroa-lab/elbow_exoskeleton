// CubeMars AK45-36 MIT Mode Control with Portenta H7 on CAN1
// CORRECTED VERSION:
//   1. Proper ExitMotorMode() using 0xFD command (was missing entirely)
//   2. State is read ONLY after entering motor mode (per datasheet requirement)
//   3. Last motor position is saved to flash/EEPROM before exiting so on the
//      next power cycle the motor corrects from its saved position to abs 0-90°
//   4. Main sweep loop runs for exactly 5 seconds then exits cleanly

// Libraries Required
#include <mbed.h>
#include "Arduino_PowerManagement.h"
#include <FlashIAPBlockDevice.h>   // Portenta H7 internal flash for state persistence

// CAN Bus Setup (pins and rate)
mbed::CAN can1(PB_8, PH_13, 1000000);

// LED Pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Motor Configuration (set from cubemars app)
#define MOTOR_ID 2
const unsigned long CAN_ID = MOTOR_ID;

// -----------------------------------------------------------------------
// Value Limits — from AK-series datasheet (section 5.3, page 44)
// Position range shared by ALL AK modules: -12.5 rad to +12.5 rad
// Speed/Torque limits below match AK10-9 column in the datasheet table.
// If you are using a different module (AK60-6, AK80-6, etc.) update
// V_MAX, T_MIN, T_MAX to match that module's column in the table.
// -----------------------------------------------------------------------
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

// -----------------------------------------------------------------------
// Outer PD Loop Gains
// -----------------------------------------------------------------------
#define OUTER_KP 1.5f
#define OUTER_KD 0.05f

// -----------------------------------------------------------------------
// Flash persistence — we store a single float (last commanded position)
// at the very end of the internal flash.  The FlashIAPBlockDevice API lets
// us read/write a raw block without an external EEPROM.
//
// Layout (8 bytes so we stay 8-byte aligned):
//   bytes 0-3 : magic cookie  0xABCD1234  (uint32_t, big-endian)
//   bytes 4-7 : last_position as IEEE-754 float
// -----------------------------------------------------------------------
#define FLASH_MAGIC   0xABCD1234UL
#define FLASH_SIZE    8            // bytes we actually use
// Place the save block in the last sector of internal flash (2 MB device,
// last 128 kB sector starts at 0x081E0000 on STM32H747).
// Using mbed FlashIAP lets us locate it portably.
static FlashIAPBlockDevice flashDev(0x081E0000, FLASH_SIZE);

// Write the last commanded position to flash so it survives a power cycle.
void savePositionToFlash(float pos) {
  if (flashDev.init() != 0) return;

  uint8_t buf[FLASH_SIZE] = {0};
  // magic
  uint32_t magic = FLASH_MAGIC;
  memcpy(buf,     &magic, 4);
  // position
  memcpy(buf + 4, &pos,   4);

  flashDev.erase(0, FLASH_SIZE);
  flashDev.program(buf, 0, FLASH_SIZE);
  flashDev.deinit();
}

// Read the last saved position.  Returns 0.0f if no valid save exists.
float loadPositionFromFlash() {
  if (flashDev.init() != 0) return 0.0f;

  uint8_t buf[FLASH_SIZE] = {0};
  flashDev.read(buf, 0, FLASH_SIZE);
  flashDev.deinit();

  uint32_t magic;
  memcpy(&magic, buf, 4);
  if (magic != FLASH_MAGIC) return 0.0f;   // no valid previous save

  float pos;
  memcpy(&pos, buf + 4, 4);
  // Sanity-check: keep within valid range
  if (pos < P_MIN || pos > P_MAX) return 0.0f;
  return pos;
}


/*********************************************************************************************************
  CONVERSION FUNCTIONS
*********************************************************************************************************/

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) pgg = (unsigned int)((x - offset) * 4095.0 / span);
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
  MOTOR MODE SECTION
  Datasheet (section 5.3, p.43) special CAN codes:
    Enter motor control mode : { 0xFF x7, 0xFC }
    Exit  motor control mode : { 0xFF x7, 0xFD }   ← was missing before
    Set current position = 0 : { 0xFF x7, 0xFE }
  NOTE: "motor control mode must be entered before using CAN communication
         control motor!" — reading state also requires the motor to be in
         motor mode first.
*********************************************************************************************************/

bool EnterMotorMode() {
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF; msg.data[1] = 0xFF; msg.data[2] = 0xFF; msg.data[3] = 0xFF;
  msg.data[4] = 0xFF; msg.data[5] = 0xFF; msg.data[6] = 0xFF; msg.data[7] = 0xFC;

  if (!can1.write(msg)) return false;

  // Send a few commands and wait for the first valid reply
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

// -----------------------------------------------------------------------
// ExitMotorMode — sends 0xFD as required by the datasheet.
// Also saves the last commanded position to flash BEFORE releasing control
// so the next power cycle can use it for the correction move.
// -----------------------------------------------------------------------
void ExitMotorMode(float lastCmdPos) {
  // 1. Save state while we still know what it is
  savePositionToFlash(lastCmdPos);

  // 2. Send Exit command (0xFD)
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF; msg.data[1] = 0xFF; msg.data[2] = 0xFF; msg.data[3] = 0xFF;
  msg.data[4] = 0xFF; msg.data[5] = 0xFF; msg.data[6] = 0xFF; msg.data[7] = 0xFD;

  can1.write(msg);
  delay(50);  // give the driver board time to acknowledge

  digitalWrite(LED_GREEN, HIGH);  // green off = no longer in motor mode
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
  moveTo — sends a position command with an outer PD correction pass.
  Returns the actual commanded position (after correction clamp) so the
  caller can track the last sent value.
*********************************************************************************************************/
float moveTo(float target_pos, uint32_t dwell_ms) {
  p_in = target_pos;

  pack_cmd();
  delay(5);

  mbed::CANMessage msgIn;
  if (can1.read(msgIn, 0)) {
    if (msgIn.id == CAN_ID && msgIn.len == 6) {
      unpack_reply(msgIn.data, msgIn.len);
    }
  }

  // Outer PD correction
  float pos_error = target_pos - p_out;
  float vel_error = 0.0f - v_out;
  float correction = constrain((OUTER_KP * pos_error) + (OUTER_KD * vel_error), -0.5f, 0.5f);
  float corrected_pos = constrain(target_pos + correction, P_MIN, P_MAX);

  p_in = corrected_pos;
  pack_cmd();
  p_in = target_pos;  // restore clean target for next call

  if (dwell_ms > 10) delay(dwell_ms - 10);

  return corrected_pos;
}


/*********************************************************************************************************
  MAIN FUNCTION SECTION
*********************************************************************************************************/

Board board;

// Track the last position we commanded so ExitMotorMode can save it.
float g_lastCmdPos = 0.0f;

// Target sweep range in radians:  0 rad (0°) → π/2 rad (90°)
#define SWEEP_MIN_RAD  0.0f
#define SWEEP_MAX_RAD  1.5708f   // π/2

// Total run time for the sweep loop (ms)
#define LOOP_DURATION_MS  5000UL


void setup() {
  board.begin();
  board.setExternalVoltage(3.3);

  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   HIGH);
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  HIGH);

  can1.mode(mbed::CAN::Mode::Normal);
  delay(1000);

  // -----------------------------------------------------------------------
  // STEP 1 — Enter motor mode.
  // Per datasheet: state cannot be read until we are in motor control mode.
  // -----------------------------------------------------------------------
  p_in = 0.0f;
  bool entered = EnterMotorMode();

  if (!entered) {
    // Flash red to signal failure; spin forever
    while (true) {
      digitalWrite(LED_RED, LOW);  delay(200);
      digitalWrite(LED_RED, HIGH); delay(200);
    }
  }

  // -----------------------------------------------------------------------
  // STEP 2 — Read the saved position from the previous power cycle (if any)
  //          and correct to it before starting the sweep.
  //          This ensures a smooth start rather than a sudden jump.
  //
  //          We dwell for 500 ms to let the motor reach the saved position
  //          before we begin the 0→90° sweep.
  // -----------------------------------------------------------------------
  float savedPos = loadPositionFromFlash();
  if (savedPos != 0.0f) {
    // Move to the previously saved position first to avoid a jerk
    g_lastCmdPos = moveTo(savedPos, 500);
    delay(300);
  }

  // -----------------------------------------------------------------------
  // STEP 3 — Go to absolute 0 rad before the timed sweep begins.
  // -----------------------------------------------------------------------
  g_lastCmdPos = moveTo(SWEEP_MIN_RAD, 1000);
  delay(500);

  lastSendTime    = millis();
  lastReceiveTime = millis();

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);
  digitalWrite(LED_RED,   LOW);
  delay(200);
}


void loop() {
  // -----------------------------------------------------------------------
  // Timed sweep: run for exactly LOOP_DURATION_MS (5 seconds), then exit
  // motor mode cleanly, saving state, and halt.
  // -----------------------------------------------------------------------
  uint32_t loopStart = millis();

  while ((millis() - loopStart) < LOOP_DURATION_MS) {

    // Sweep forward 0 → π/2 rad
    for (float pos = SWEEP_MIN_RAD; pos <= SWEEP_MAX_RAD; pos += 0.005f) {
      if ((millis() - loopStart) >= LOOP_DURATION_MS) goto done_sweep;
      g_lastCmdPos = moveTo(pos, 20);
    }

    // Sweep back π/2 → 0 rad
    for (float pos = SWEEP_MAX_RAD; pos >= SWEEP_MIN_RAD; pos -= 0.005f) {
      if ((millis() - loopStart) >= LOOP_DURATION_MS) goto done_sweep;
      g_lastCmdPos = moveTo(pos, 20);
    }
  }

  done_sweep:

  // -----------------------------------------------------------------------
  // STEP 4 — Exit motor mode correctly (0xFD) and save the last position.
  //
  // ExitMotorMode() calls savePositionToFlash() FIRST (while the value is
  // still valid), then sends the 0xFD CAN frame.
  // On the next power cycle, setup() will load this position and move to it
  // before starting the sweep, giving continuity across resets.
  // -----------------------------------------------------------------------
  ExitMotorMode(g_lastCmdPos);

  // Signal completion: solid blue
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED,   HIGH);

  // Halt — do not re-enter loop()
  while (true) { delay(1000); }
}


/*********************************************************************************************************
  END FILE
  -------------------------------------------------------------------------
  CHANGE SUMMARY vs original:
  1. ExitMotorMode(float lastCmdPos) added — sends 0xFD per datasheet §5.3
     and calls savePositionToFlash() BEFORE releasing the motor.
  2. EnterMotorMode() is now called BEFORE any attempt to read motor state.
     The datasheet states: "motor control mode must be entered before using
     CAN communication control motor" — this applies to state reads too.
  3. Flash persistence (savePositionToFlash / loadPositionFromFlash) stores
     the last commanded position in the Portenta H7's internal flash using
     FlashIAPBlockDevice.  On each power cycle setup() loads this value and
     moves the motor to it before transitioning to the 0→90° sweep.
  4. Main sweep loop is now bounded by LOOP_DURATION_MS (5 000 ms).  A
     goto exits any inner for-loop the moment the 5-second window closes,
     then ExitMotorMode is called unconditionally.
  5. Sweep range changed from 0–1.57 rad (fine for 90°) — now named
     constants SWEEP_MIN_RAD / SWEEP_MAX_RAD make it easy to adjust.
*********************************************************************************************************/

// Exercise the production serial/wait and motor code with a fake clock and GPIO.
// ArduinoJson is the same pinned header-only library used by the firmware build.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include <vector>
#include <ArduinoJson.h>
#include "../General_Driver/bot_runtime.h"
#include "../General_Driver/json_cmd.h"

using String = std::string;
using byte = uint8_t;
static uint32_t nowMs;
uint32_t millis() { return nowMs; }
uint32_t micros() { return nowMs * 1000; }
void delay(uint32_t duration) { nowMs += duration; }

struct Port {
  std::deque<char> input;
  size_t reads = 0;
  uint32_t releaseAt = 0;
  void write(const std::string& text) { input.insert(input.end(), text.begin(), text.end()); }
  int available() const { return nowMs >= releaseAt ? input.size() : 0; }
  char read() { char ch = input.front(); input.pop_front(); ++reads; return ch; }
  template <class T> void println(const T&) {}
} Serial;

enum { LOW, HIGH, OUTPUT };
enum { AIN1, AIN2, PWMA, BIN1, BIN2, PWMB };
int pins[6] = {}, pwm[2] = {};
void pinMode(int, int) {}
void digitalWrite(int pin, int value) { pins[pin] = value; }
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
void ledcWrite(int channel, int value) { pwm[channel] = value; }
constexpr int channel_A = 0, channel_B = 1, freq = 1000, ANALOG_WRITE_BITS = 8;
constexpr int AENCA = 0, AENCB = 1, BENCA = 2, BENCB = 3;
constexpr int THRESHOLD_PWM = 0;
double WHEEL_D = 0.08, ONE_CIRCLE_PLUSES = 2100, TRACK_WIDTH = 0.125;
double __kp = 1, __ki = 0, __kd = 0, windup_limits = 255;
bool SET_MOTOR_DIR = false;
byte mainType = 1, moduleType = 0;
String screenLine_2;
uint32_t lastCmdRecvTime;
int HEART_BEAT_DELAY = BOT_HEARTBEAT_MS;
struct ESP32Encoder {
  void attachHalfQuad(int, int) {}
  void setCount(int) {}
  long getCount() const { return 0; }
};
namespace PID { enum { Direct, Automatic }; }
struct PID_v2 {
  PID_v2(double, double, double, int) {}
  void Start(double, double, double) {}
  void SetOutputLimits(int, int) {}
  void SetMode(int) {}
  void Setpoint(double) {}
  void SetTunings(double, double, double) {}
  double Run(double) const { return 0; }
};

StaticJsonDocument<512> jsonInfoHttp;
StaticJsonDocument<256> jsonCmdReceive;
uint8_t bot_stopFlags;
uint16_t bot_clamp_count;
#include "../General_Driver/movtion_module.h"

int InfoPrint = 0;
bool uartCmdEcho = false;
unsigned safetyPolls;
void bot_safetyUpdate() { ++safetyPolls; }
std::vector<int> dispatched;
uint32_t dispatchedReceipt;
// Vendor dispatch is not emulated: record delivery to prove ordering and that
// nested wait polls do not overwrite its global JSON document.
void jsonCmdReceiveHandler(uint32_t receivedAt = millis()) {
  dispatched.push_back(jsonCmdReceive["T"].as<int>());
  dispatchedReceipt = receivedAt;
  if (jsonCmdReceive["T"] == 111) bot_delayMillis(jsonCmdReceive["cmd"]);
}
#include "../General_Driver/bot_serial_ctrl.h"

void reset() {
  nowMs = 0;
  Serial = Port{};
  bot_serialInput = BotSerialInput{};
  bot_waitStopped = false;
  bot_stopGeneration = 0;
  mainType = 1;
  moduleType = 0;
  bot_stopFlags = 0;
  bot_clamp_count = 0;
  heartbeatStopFlag = false;
  lastCmdRecvTime = 0;
  HEART_BEAT_DELAY = BOT_HEARTBEAT_MS;
  safetyPolls = 0;
  dispatched.clear();
  dispatchedReceipt = 0;
  bot_stopMotors();
}

void assertStopped() {
  assert(pwm[0] == 0 && pwm[1] == 0);
  assert(bot_req_A == 0 && bot_req_B == 0);
  assert(bot_applied_L == 0 && bot_applied_R == 0);
  assert(setpointA == 0 && setpointB == 0 && !usePIDCompute);
}

int main() {
  reset();
  setGoalSpeed(0.2f, 0.2f);
  assert(pwm[0] > 0 && pwm[1] > 0);
  mm_settings(3, 0);
  assert(pwm[0] == 0 && pwm[1] == 0);  // mode change removes residual PWM
  leftCtrl(100);
  rightCtrl(100);  // raw PWM still works in mode 3
  nowMs = 299;
  heartBeatCtrl();
  assert(pwm[0] == 100);
  nowMs = 300;
  heartBeatCtrl();
  assertStopped();  // heartbeat must not depend on disabled PID execution

  reset();
  mainType = 3;
  leftCtrl(100);
  rightCtrl(100);
  setGoalSpeed(0, 0);
  assertStopped();
  mainType = 1;
  setGoalSpeed(0.2f, 0.2f);
  mm_settings(1, 2);
  assert(pwm[0] > 0);  // a module-only change does not stop normal motion
  bot_onCoast();
  assertStopped();
  assert(pins[AIN1] == LOW && pins[AIN2] == LOW);
  assert(pins[BIN1] == LOW && pins[BIN2] == LOW);

  reset();
  leftCtrl(255);
  rightCtrl(-255);
  assert(pwm[0] == 154 && pwm[1] == 154 && bot_clamp_count == 2);
  bot_stopFlags = BOT_ST_TOF;
  setGoalSpeed(0.2f, 0.2f);
  assert(pwm[0] == 0 && pwm[1] == 0);
  setGoalSpeed(-0.2f, -0.2f);
  assert(pwm[0] > 0 && pwm[1] > 0);
  bot_stopFlags = BOT_ST_LOWBAT;
  bot_driveMotors();
  assert(pwm[0] == 0 && pwm[1] == 0);

  reset();
  setGoalSpeed(0.2f, 0.2f);
  bot_delayMillis(3000);
  assert(nowMs == 3000 && safetyPolls == 3000);
  assertStopped();
  assert(heartbeatStopFlag);

  // Expired wheel commands must not restart motion when a vendor wait ends.
  // Other vendor operations keep their ordering and are not heartbeat-limited.
  for (const char* motion : {"{\"T\":1,\"L\":0.2,\"R\":0.2}\n",
                             "{\"T\":11,\"L\":100,\"R\":100}\n",
                             "{\"T\":13,\"X\":0.2,\"Z\":0}\n"}) {
    for (uint32_t receivedAt : {0u, 500u}) {
      reset();
      setGoalSpeed(0.2f, 0.2f);
      Serial.releaseAt = receivedAt;
      Serial.write(std::string(motion) + "{\"T\":130}\n");
      bot_delayMillis(3000);
      assertStopped();
      while (bot_pendingSerial()) serialCtrl();
      assert((dispatched == std::vector<int>{130}));
      assert(bot_serialInput.dropped == 1);
      assertStopped();
    }
  }

  reset();
  nowMs = 100;
  Serial.write("{\"T\":1,\"L\":0.2,\"R\":0.2}\n");
  bot_delayMillis(299);
  serialCtrl();
  assert((dispatched == std::vector<int>{1}));
  assert(dispatchedReceipt == 100);  // dispatch does not renew a deferred command
  assert(bot_serialInput.dropped == 0);

  reset();
  nowMs = UINT32_MAX - 100;
  Serial.write("{\"T\":1,\"L\":0.2,\"R\":0.2}\n");
  bot_delayMillis(300);
  serialCtrl();
  assert(dispatched.empty());  // exactly expired, even across millis() wrap
  assert(bot_serialInput.dropped == 1);

  reset();
  setGoalSpeed(0.2f, 0.2f);
  Serial.write("{\"T\":1,\"L\":0.3,\"R\":0.3}\n{\"T\":115}\n");
  jsonCmdReceive["T"] = 111;
  jsonCmdReceive["cmd"] = 3000;
  bot_delayMillis(3000);
  assert(nowMs < 300 && bot_waitStopped);
  assertStopped();
  assert(!bot_pendingSerial());  // stopped motion cannot restart from the wait queue
  assert(jsonCmdReceive["T"].as<int>() == 111);
  assert(jsonCmdReceive["cmd"].as<int>() == 3000);

  reset();
  setGoalSpeed(0.2f, 0.2f);
  Serial.releaseAt = 50;
  Serial.write("{\"T\":1,\"L\":0,\"R\":0}\n");
  bot_delayMillis(3000);
  assert(nowMs >= 50 && nowMs < 55);
  assertStopped();

  for (const char* command : {"{\"T\":11,\"L\":0,\"R\":0}\n",
                              "{\"T\":13,\"X\":0,\"Z\":0}\n"}) {
    reset();
    setGoalSpeed(0.2f, 0.2f);
    Serial.write(command);
    bot_delayMillis(3000);
    assert(nowMs < 5);
    assertStopped();
  }

  reset();
  Serial.write("{\"T\":3}\n{\"T\":130}\n");
  bot_delayMillis(10);
  assert(dispatched.empty());
  serialCtrl();
  serialCtrl();
  assert((dispatched == std::vector<int>{3, 130}));

  reset();
  Serial.write("{\"T\":111,\"cmd\":10}\n{\"T\":130");
  serialCtrl();  // re-enter intake during dispatch, leaving a partial next line
  assert(jsonCmdReceive["T"].as<int>() == 111);
  Serial.write("}\n");
  serialCtrl();
  assert((dispatched == std::vector<int>{111, 130}));

  reset();
  Serial.write(std::string(200000, 'x') + "\n{\"T\":130}\n");
  while (Serial.available()) {
    size_t before = Serial.reads;
    serialCtrl();
    assert(Serial.reads - before <= BOT_SERIAL_BYTES_PER_POLL);
  }
  assert(bot_serialInput.dropped == 1);
  assert((dispatched == std::vector<int>{130}));

  reset();
  for (int i = 0; i < 20; ++i) Serial.write("{\"T\":3}\n");
  bot_delayMillis(30);
  assert(bot_serialInput.dropped == 20 - BOT_SERIAL_PENDING_LINES);
  while (bot_pendingSerial()) serialCtrl();
  assert(dispatched.size() == BOT_SERIAL_PENDING_LINES);

  reset();
  nowMs = UINT32_MAX - 100;
  lastCmdRecvTime = nowMs;
  setGoalSpeed(0.2f, 0.2f);
  bot_delayMillis(300);
  heartBeatCtrl();
  assertStopped();  // timeout arithmetic also works across millis() wrap

  std::cout << "PASS firmware motor stops, bounded serial intake, cooperative waits\n";
}

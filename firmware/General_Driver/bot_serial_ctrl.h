// Serial dispatch and cooperative waits; included after the vendor command handler.
BotSerialInput bot_serialInput;
bool bot_waitStopped = false;
uint32_t bot_stopGeneration = 0;

bool bot_pendingSerial() { return bot_serialInput.front() != nullptr; }

// Parse urgent stops separately: vendor functions can still be using the
// global command document while their cooperative wait services the serial line.
bool bot_tryImmediateStop(const char* line, size_t size) {
  StaticJsonDocument<256> stop;
  if (deserializeJson(stop, line, size)) return false;
  int type = stop["T"].as<int>();
  bool zero = type == CMD_SPEED_CTRL && stop["L"].is<float>() &&
              stop["R"].is<float>() && stop["L"].as<float>() == 0 &&
              stop["R"].as<float>() == 0;
  zero = zero || (type == CMD_PWM_INPUT && stop["L"].as<float>() == 0 &&
                  stop["R"].as<float>() == 0);
  zero = zero || (type == CMD_ROS_CTRL && stop["X"].as<float>() == 0 &&
                  stop["Z"].as<float>() == 0);
  if (type != CMD_SWITCH_OFF && !zero) return false;
  if (zero) {
    heartbeatStopFlag = false;
    lastCmdRecvTime = millis();
    bot_stopFlags &= ~BOT_ST_COAST;
    bot_stopMotors();
  } else {
    bot_onCoast();
  }
  // A stop cancels earlier queued commands; none can restart motion after it.
  bot_serialInput.cancelPending();
  bot_waitStopped = true;
  ++bot_stopGeneration;
  return true;
}

void bot_handleSerialLine(const char* line, size_t size) {
  if (deserializeJson(jsonCmdReceive, line, size)) return;
  if (InfoPrint == 1 && uartCmdEcho) Serial.println(line);
  bot_waitStopped = false;
  jsonCmdReceiveHandler();
}

void serialCtrl() {
  if (const char* pending = bot_serialInput.front()) {
    // Parsing copies strings into jsonCmdReceive before a nested wait can
    // overwrite the vacated queue slot.
    if (!deserializeJson(jsonCmdReceive, pending)) {
      if (InfoPrint == 1 && uartCmdEcho) Serial.println(pending);
      uint32_t receivedAt = bot_serialInput.frontReceivedAt();
      bot_serialInput.pop();
      int type = jsonCmdReceive["T"].as<int>();
      bool motion = type == CMD_SPEED_CTRL || type == CMD_PWM_INPUT || type == CMD_ROS_CTRL;
      if (motion && (uint32_t)(millis() - receivedAt) >= (uint32_t)HEART_BEAT_DELAY) {
        ++bot_serialInput.dropped;
        return;  // a vendor wait cannot extend a queued wheel command's lifetime
      }
      bot_waitStopped = false;
      // Accepted wheel commands retain only the heartbeat time left at receipt.
      jsonCmdReceiveHandler(receivedAt);
      heartBeatCtrl();
    } else {
      bot_serialInput.pop();
    }
    return;
  }
  bot_serialInput.poll(Serial, bot_handleSerialLine);
}

void bot_serviceSafety() {
  heartBeatCtrl();
  bot_safetyUpdate();
  bot_serialInput.poll(Serial, [](const char* line, size_t size) {
    if (!bot_tryImmediateStop(line, size)) bot_serialInput.defer(line, size, millis());
  });
  heartBeatCtrl();
}

void bot_delayMillis(uint32_t duration) {
  uint32_t start = millis();
  uint32_t generation = bot_stopGeneration;
  while ((uint32_t)(millis() - start) < duration && generation == bot_stopGeneration) {
    bot_serviceSafety();
    delay(1);  // yield to ESP32 tasks; never suspend the safety checks for a full wait
  }
}

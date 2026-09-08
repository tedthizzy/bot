// Bounded serial intake shared by the normal loop and cooperative vendor waits.
#ifndef BOT_RUNTIME_H
#define BOT_RUNTIME_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "bot_config.h"

class BotSerialInput {
 public:
  // At most one complete line and BOT_SERIAL_BYTES_PER_POLL bytes per call.
  template <class Port, class Handler>
  void poll(Port& port, Handler handle) {
    for (size_t n = 0; n < BOT_SERIAL_BYTES_PER_POLL && port.available(); ++n) {
      char ch = port.read();
      if (ch == '\n') {
        if (discarding_) {
          ++dropped;
        } else {
          if (used_ && line_[used_ - 1] == '\r') --used_;
          line_[used_] = '\0';
          size_t size = used_;
          used_ = 0;  // a handler may service serial again during a vendor wait
          handle(line_, size);
          return;
        }
        used_ = 0;
        discarding_ = false;
        return;
      }
      if (discarding_) continue;
      if (used_ == BOT_SERIAL_LINE_MAX) {
        discarding_ = true;  // discard the entire oversized line, through LF
      } else {
        line_[used_++] = ch;
      }
    }
  }

  bool defer(const char* line, size_t size, uint32_t receivedAt) {
    if (count_ == BOT_SERIAL_PENDING_LINES) {
      ++dropped;
      return false;
    }
    size_t slot = (first_ + count_) % BOT_SERIAL_PENDING_LINES;
    memcpy(pending_[slot], line, size + 1);
    receivedAt_[slot] = receivedAt;
    ++count_;
    return true;
  }

  const char* front() const { return count_ ? pending_[first_] : nullptr; }
  uint32_t frontReceivedAt() const { return receivedAt_[first_]; }
  void pop() {
    if (count_) {
      first_ = (first_ + 1) % BOT_SERIAL_PENDING_LINES;
      --count_;
    }
  }
  void cancelPending() { dropped += count_; count_ = 0; }
  uint32_t dropped = 0;

 private:
  char line_[BOT_SERIAL_LINE_MAX + 1] = {};
  char pending_[BOT_SERIAL_PENDING_LINES][BOT_SERIAL_LINE_MAX + 1] = {};
  uint32_t receivedAt_[BOT_SERIAL_PENDING_LINES] = {};
  size_t used_ = 0, first_ = 0, count_ = 0;
  bool discarding_ = false;
};

void bot_serviceSafety();
void bot_delayMillis(uint32_t duration);
bool bot_pendingSerial();
extern bool bot_waitStopped;

#endif

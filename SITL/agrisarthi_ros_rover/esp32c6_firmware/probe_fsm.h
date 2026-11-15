#pragma once

#include <stdint.h>
#include <string.h>

enum class ProbeState : uint8_t {
  IDLE, INSERTING, INSERTED, RETRACTING, RETRACTED, ERROR
};

inline const char* probeStateName(ProbeState state) {
  switch (state) {
    case ProbeState::IDLE: return "IDLE";
    case ProbeState::INSERTING: return "INSERTING";
    case ProbeState::INSERTED: return "INSERTED";
    case ProbeState::RETRACTING: return "RETRACTING";
    case ProbeState::RETRACTED: return "RETRACTED";
    default: return "ERROR";
  }
}

struct ProbeCommandResult {
  bool accepted;
  bool duplicate;
  bool beginInsert;
  bool beginRetract;
};

class ProbeStateMachine {
 public:
  ProbeStateMachine() : state_(ProbeState::IDLE), moveStarted_(0), moveDuration_(0),
      settleDuration_(0), commandTimeout_(0), historyIndex_(0), positionAssumed_(true) {
    error_[0] = '\0';
    activeCommandId_[0] = '\0';
    for (uint8_t i = 0; i < kHistorySize; ++i) {
      history_[i].id[0] = '\0';
      history_[i].action[0] = '\0';
      history_[i].accepted = false;
    }
  }

  ProbeState state() const { return state_; }
  const char* error() const { return error_; }
  const char* activeCommandId() const { return activeCommandId_; }
  bool motorAllowed() const { return state_ == ProbeState::RETRACTED; }
  bool positionAssumed() const { return positionAssumed_; }
  bool cachedResult(const char* commandId, const char* action, bool& accepted) const {
    if (!validId(commandId) || action == nullptr) return false;
    int8_t cached = findId(commandId);
    if (cached < 0) return false;
    accepted = strcmp(history_[cached].action, action) == 0 && history_[cached].accepted;
    return true;
  }

  void setConfigurationError(const char* message) {
    state_ = ProbeState::ERROR;
    copyText(error_, sizeof(error_), message);
  }

  void setBootRetracted() {
    state_ = ProbeState::RETRACTED;
    error_[0] = '\0';
    positionAssumed_ = false;
  }

  ProbeCommandResult command(const char* commandId, const char* action, uint32_t now,
                            uint32_t travelMs, uint32_t settleMs, uint32_t timeoutMs,
                            bool retractLimitActive = false) {
    ProbeCommandResult result = {false, false, false, false};
    if (!validId(commandId) || action == nullptr || strlen(action) == 0 || strlen(action) > 20) {
      copyText(error_, sizeof(error_), "INVALID_COMMAND");
      return result;
    }
    int8_t cached = findId(commandId);
    if (cached >= 0) {
      result.duplicate = true;
      result.accepted = strcmp(history_[cached].action, action) == 0 && history_[cached].accepted;
      if (strcmp(history_[cached].action, action) != 0) copyText(error_, sizeof(error_), "COMMAND_ID_CONFLICT");
      return result;
    }

    if (strcmp(action, "PROBE_STATUS") == 0) {
      result.accepted = true;
      remember(commandId, action, true);
      copyText(activeCommandId_, sizeof(activeCommandId_), commandId);
      return result;
    }
    if (strcmp(action, "PROBE_INSERT") == 0) {
      if (state_ == ProbeState::INSERTED) {
        result.accepted = true;
        copyText(activeCommandId_, sizeof(activeCommandId_), commandId);
      } else if (state_ == ProbeState::RETRACTED) {
        beginMove(ProbeState::INSERTING, commandId, action, now, travelMs, settleMs, timeoutMs);
        result.accepted = true;
        result.beginInsert = true;
      } else {
        copyText(error_, sizeof(error_), state_ == ProbeState::ERROR || state_ == ProbeState::IDLE
                                         ? "RETRACT_REQUIRED" : "ACTUATOR_BUSY");
      }
      remember(commandId, action, result.accepted);
      return result;
    }
    if (strcmp(action, "PROBE_RETRACT") == 0) {
      if (state_ == ProbeState::RETRACTED || retractLimitActive) {
        state_ = ProbeState::RETRACTED;
        error_[0] = '\0';
        positionAssumed_ = !retractLimitActive;
        copyText(activeCommandId_, sizeof(activeCommandId_), commandId);
        result.accepted = true;
      } else if (state_ == ProbeState::INSERTED || state_ == ProbeState::ERROR ||
                 state_ == ProbeState::IDLE || state_ == ProbeState::INSERTING) {
        beginMove(ProbeState::RETRACTING, commandId, action, now, travelMs, settleMs, timeoutMs);
        result.accepted = true;
        result.beginRetract = true;
      } else {
        copyText(error_, sizeof(error_), "ACTUATOR_BUSY");
      }
      remember(commandId, action, result.accepted);
      return result;
    }
    copyText(error_, sizeof(error_), "INVALID_COMMAND");
    remember(commandId, action, false);
    return result;
  }

  bool tick(uint32_t now, uint32_t insertLimitGraceMs, bool useLimitSwitches,
            bool insertLimitActive, bool retractLimitActive) {
    if (state_ != ProbeState::INSERTING && state_ != ProbeState::RETRACTING) return false;
    if (static_cast<uint32_t>(now - moveStarted_) > commandTimeout_) {
      state_ = ProbeState::ERROR;
      copyText(error_, sizeof(error_), "SERVO_TIMEOUT");
      return true;
    }
    const bool inserting = state_ == ProbeState::INSERTING;
    const bool reached = useLimitSwitches ? (inserting ? insertLimitActive : retractLimitActive)
                                          : static_cast<uint32_t>(now - moveStarted_) >= moveDuration_ + settleDuration_;
    if (reached) {
      state_ = inserting ? ProbeState::INSERTED : ProbeState::RETRACTED;
      error_[0] = '\0';
      positionAssumed_ = !useLimitSwitches;
      return true;
    }
    if (useLimitSwitches && static_cast<uint32_t>(now - moveStarted_) >= moveDuration_ + settleDuration_ + insertLimitGraceMs) {
      state_ = ProbeState::ERROR;
      copyText(error_, sizeof(error_), inserting ? "INSERT_LIMIT_NOT_REACHED" : "RETRACT_LIMIT_NOT_REACHED");
      return true;
    }
    return false;
  }

 private:
  static constexpr uint8_t kHistorySize = 32;
  struct CachedCommand { char id[65]; char action[21]; bool accepted; };
  ProbeState state_;
  uint32_t moveStarted_;
  uint32_t moveDuration_;
  uint32_t settleDuration_;
  uint32_t commandTimeout_;
  uint8_t historyIndex_;
  bool positionAssumed_;
  char error_[48];
  char activeCommandId_[65];
  CachedCommand history_[kHistorySize];

  static bool validId(const char* value) {
    return value != nullptr && strlen(value) > 0 && strlen(value) <= 64;
  }
  static void copyText(char* destination, size_t capacity, const char* source) {
    if (capacity == 0) return;
    if (source == nullptr) source = "";
    strncpy(destination, source, capacity - 1);
    destination[capacity - 1] = '\0';
  }
  int8_t findId(const char* id) const {
    for (uint8_t i = 0; i < kHistorySize; ++i) {
      if (history_[i].id[0] != '\0' && strcmp(history_[i].id, id) == 0) return static_cast<int8_t>(i);
    }
    return -1;
  }
  void remember(const char* id, const char* action, bool accepted) {
    CachedCommand& entry = history_[historyIndex_];
    copyText(entry.id, sizeof(entry.id), id);
    copyText(entry.action, sizeof(entry.action), action);
    entry.accepted = accepted;
    historyIndex_ = (historyIndex_ + 1) % kHistorySize;
  }
  void beginMove(ProbeState next, const char* id, const char* action, uint32_t now,
                 uint32_t travelMs, uint32_t settleMs, uint32_t timeoutMs) {
    state_ = next;
    moveStarted_ = now;
    moveDuration_ = travelMs;
    settleDuration_ = settleMs;
    commandTimeout_ = timeoutMs;
    error_[0] = '\0';
    positionAssumed_ = true;
    copyText(activeCommandId_, sizeof(activeCommandId_), id);
    (void)action;
  }
};

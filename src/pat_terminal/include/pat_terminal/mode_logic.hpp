#pragma once

#include <cstdint>

namespace pat_terminal {

enum class Mode : uint8_t {
  IDLE = 0,
  ACQUIRE = 1,
  HANDOFF = 2,
  LOCK = 3,
  COAST = 4,
  SAFE = 5,
};

struct ModeParams {
  double lock_error_threshold;  // [rad] HANDOFF/COAST -> LOCK error criterion
  double lock_debounce_s;       // error must stay below threshold this long
  double handoff_timeout_s;     // HANDOFF -> ACQUIRE abort
  double coast_entry_s;         // LOCK -> COAST after this long without a valid spot
  double coast_timeout_s;       // COAST -> ACQUIRE fallback
};

// Overall state machine for the PAT terminal 
class ModeLogic {
public:
  explicit ModeLogic(const ModeParams & params)
  : params_(params) {}

  Mode mode() const {return mode_;}

  /**
  * @brief Interface for the host to set mode
  * @returns true if legal, false if illegal
  */
  bool request(Mode target) {
    const bool legal = mode_ == Mode::IDLE && target == Mode::ACQUIRE;
    if (legal) {
      mode_ = target;
    }
    return legal;
  }

private:
  ModeParams params_;
  Mode mode_{Mode::IDLE};
};

}  // namespace pat_terminal

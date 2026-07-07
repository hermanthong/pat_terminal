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

struct ModeInputs {
  double dt;        // [s] since last tick
  bool spot_valid;  // fresh valid camera frame this tick
  double error;     // [rad] estimated position error magnitude
};

// Overall state machine for the PAT terminal
class ModeLogic {
public:
  explicit ModeLogic(const ModeParams & params)
  : params_(params) {}

  Mode mode() const {return mode_;}

  Mode tick(const ModeInputs & inputs) {
    switch (mode_) {
      case Mode::ACQUIRE:
        if (inputs.spot_valid) {
          enter(Mode::HANDOFF);
        }
        break;
      case Mode::HANDOFF:
        time_in_mode_s_ += inputs.dt;
        update_lock_debounce(inputs);
        if (lock_debounce_s_ >= params_.lock_debounce_s) {
          enter(Mode::LOCK);
        } else if (time_in_mode_s_ >= params_.handoff_timeout_s) {
          enter(Mode::ACQUIRE);
        }
        break;
      case Mode::LOCK:
        // Continuous time without a valid spot, any valid frame restarts it.
        if (inputs.spot_valid) {
          spot_loss_s_ = 0.0;
        } else {
          spot_loss_s_ += inputs.dt;
        }
        if (spot_loss_s_ >= params_.coast_entry_s) {
          enter(Mode::COAST);
        }
        break;
      case Mode::COAST:
        time_in_mode_s_ += inputs.dt;
        update_lock_debounce(inputs);
        if (lock_debounce_s_ >= params_.lock_debounce_s) {
          enter(Mode::LOCK);
        } else if (time_in_mode_s_ >= params_.coast_timeout_s) {
          enter(Mode::ACQUIRE);
        }
        break;
      case Mode::SAFE:
      // do nothing. only way out is for the host to command it.
      default:
        break;
    }
    return mode_;
  }

  /**
  * @brief Interface for the host to set mode
  * @returns true if legal, false if illegal
  */
  bool request(Mode target) {
    const bool legal = target == Mode::SAFE ||
      (mode_ == Mode::IDLE && target == Mode::ACQUIRE) ||
      (mode_ == Mode::SAFE && target == Mode::IDLE);
    if (legal) {
      mode_ = target;
    }
    return legal;
  }

private:
  void update_lock_debounce(const ModeInputs & inputs) {
    if (inputs.spot_valid && inputs.error < params_.lock_error_threshold) {
      lock_debounce_s_ += inputs.dt;
    } else {
      lock_debounce_s_ = 0.0;
    }
  }

  void enter(Mode mode) {
    mode_ = mode;
    lock_debounce_s_ = 0.0;
    time_in_mode_s_ = 0.0;
    spot_loss_s_ = 0.0;
  }

  ModeParams params_;
  Mode mode_{Mode::IDLE};
  double lock_debounce_s_{0.0};
  double time_in_mode_s_{0.0};
  double spot_loss_s_{0.0};
};

}  // namespace pat_terminal

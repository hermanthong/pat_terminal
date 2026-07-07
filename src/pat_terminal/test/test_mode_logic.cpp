#include <gtest/gtest.h>
#include "pat_terminal/mode_logic.hpp"

using pat_terminal::Mode;
using pat_terminal::ModeInputs;
using pat_terminal::ModeLogic;
using pat_terminal::ModeParams;

namespace {

ModeParams params() {
  return ModeParams{
    .lock_error_threshold = 50e-6,
    .lock_debounce_s = 0.200,
    .handoff_error_threshold = 0.5e-3,
    .handoff_timeout_s = 2.0,
    .coast_entry_s = 0.100,
    .coast_timeout_s = 0.500,
  };
}

}  // namespace

TEST(ModeLogic, StartsIdleAndAcceptsAcquireRequest) {
  ModeLogic logic(params());
  EXPECT_EQ(logic.mode(), Mode::IDLE);
  EXPECT_TRUE(logic.request(Mode::ACQUIRE));
  EXPECT_EQ(logic.mode(), Mode::ACQUIRE);
}

TEST(ModeLogic, HostCanAlwaysCommandSafe) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  EXPECT_TRUE(logic.request(Mode::SAFE));
  EXPECT_EQ(logic.mode(), Mode::SAFE);
}

TEST(ModeLogic, AcquireEntersHandoffConditions) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = false, .error = 0.0});
  EXPECT_EQ(logic.mode(), Mode::ACQUIRE);

  // error is too high for FSM
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 3e-3});
  EXPECT_EQ(logic.mode(), Mode::ACQUIRE);

  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});
  EXPECT_EQ(logic.mode(), Mode::HANDOFF);
}

TEST(ModeLogic, HandoffLocksOnlyAfterDebouncedLowError) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});
  EXPECT_EQ(logic.mode(), Mode::HANDOFF);

  // t = 180 ms, not yet debounced.
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = true, .error = 20e-6});
    EXPECT_EQ(logic.mode(), Mode::HANDOFF);
  }
  // t > 200ms, debouncing should occur
  logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  EXPECT_EQ(logic.mode(), Mode::LOCK);
}

TEST(ModeLogic, ErrorSpikeRestartsLockDebounce) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});
  EXPECT_EQ(logic.mode(), Mode::HANDOFF);

  // t = 180 ms
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = true, .error = 20e-6});
  }

  // error spike resets debounce
  logic.tick({.dt = 0.020, .spot_valid = true, .error = 200e-6});
  EXPECT_EQ(logic.mode(), Mode::HANDOFF);

  // t =  180 ms
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = true, .error = 20e-6});
    EXPECT_EQ(logic.mode(), Mode::HANDOFF);
  }

  // t > 200ms
  logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  EXPECT_EQ(logic.mode(), Mode::LOCK);
}

TEST(ModeLogic, HandoffAbortsToAcquireOnTimeout) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});

  // t = 1.9 s
  for (int i = 0; i < 19; ++i) {
    logic.tick({.dt = 0.100, .spot_valid = true, .error = 200e-6});
    EXPECT_EQ(logic.mode(), Mode::HANDOFF);
  }

  // t > 2 s
  logic.tick({.dt = 0.150, .spot_valid = true, .error = 200e-6});
  EXPECT_EQ(logic.mode(), Mode::ACQUIRE);
}

TEST(ModeLogic, LockCoastsOnlyAfterContinuousSpotLoss) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});
  // t > 200 ms of low error, debounce complete
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  }
  ASSERT_EQ(logic.mode(), Mode::LOCK);

  // t < 100 ms 
  for (int i = 0; i < 4; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = false, .error = 0.0});
    EXPECT_EQ(logic.mode(), Mode::LOCK);
  }

  // t = 0, spot_loss_s_ is reset by a valid spot
  logic.tick({.dt = 0.020, .spot_valid = true, .error = 20e-6});
  EXPECT_EQ(logic.mode(), Mode::LOCK);

  // t < 100 ms
  for (int i = 0; i < 4; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = false, .error = 0.0});
    EXPECT_EQ(logic.mode(), Mode::LOCK);
  }

  // t > 100 ms
  logic.tick({.dt = 0.025, .spot_valid = false, .error = 0.0});
  EXPECT_EQ(logic.mode(), Mode::COAST);
}

TEST(ModeLogic, LockFallsBackToAcquireOnDivergedError) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});
  // t > 200 ms of low error, debounce complete
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  }
  ASSERT_EQ(logic.mode(), Mode::LOCK);

  // a brief spike should not break the lock
  logic.tick({.dt = 0.020, .spot_valid = true, .error = 3e-3});
  EXPECT_EQ(logic.mode(), Mode::LOCK);
  logic.tick({.dt = 0.020, .spot_valid = true, .error = 20e-6});
  EXPECT_EQ(logic.mode(), Mode::LOCK);

  // t = 180 ms
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = true, .error = 3e-3});
    EXPECT_EQ(logic.mode(), Mode::LOCK);
  }
  // t > 200 ms, lock should break
  logic.tick({.dt = 0.025, .spot_valid = true, .error = 3e-3});
  EXPECT_EQ(logic.mode(), Mode::ACQUIRE);
}

TEST(ModeLogic, CoastRelocksOnDebouncedSpotReturn) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});

  // t > 200 ms of low error, debounce complete
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  }
  ASSERT_EQ(logic.mode(), Mode::LOCK);

  // t < 100 ms without the spot
  for (int i = 0; i < 4; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = false, .error = 0.0});
    ASSERT_EQ(logic.mode(), Mode::LOCK);
  }

  // t > 100 ms without the spot
  logic.tick({.dt = 0.025, .spot_valid = false, .error = 0.0});
  ASSERT_EQ(logic.mode(), Mode::COAST);

  // t > 200ms with the spot
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = true, .error = 20e-6});
    EXPECT_EQ(logic.mode(), Mode::COAST);
  }

  logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  EXPECT_EQ(logic.mode(), Mode::LOCK);
}

TEST(ModeLogic, CoastFallsBackToAcquireOnTimeout) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});

  // t > 200 ms of low error, debounce complete
  for (int i = 0; i < 9; ++i) {
    logic.tick({.dt = 0.025, .spot_valid = true, .error = 20e-6});
  }

  // t > 100 ms without the spot, COAST entered on the final tick
  for (int i = 0; i < 4; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = false, .error = 0.0});
  }

  logic.tick({.dt = 0.025, .spot_valid = false, .error = 0.0});
  ASSERT_EQ(logic.mode(), Mode::COAST);

  // t = 480 ms in COAST, the spot never returns
  for (int i = 0; i < 24; ++i) {
    logic.tick({.dt = 0.020, .spot_valid = false, .error = 0.0});
    EXPECT_EQ(logic.mode(), Mode::COAST);
  }

  // t > 500 ms
  logic.tick({.dt = 0.025, .spot_valid = false, .error = 0.0});
  EXPECT_EQ(logic.mode(), Mode::ACQUIRE);
}

TEST(ModeLogic, SafeIgnoresTicks) {
  ModeLogic logic(params());
  logic.request(Mode::ACQUIRE);
  logic.tick({.dt = 1.0 / 60.0, .spot_valid = true, .error = 300e-6});
  EXPECT_TRUE(logic.request(Mode::SAFE));

  logic.tick({.dt = 1.0, .spot_valid = true, .error = 20e-6});
  logic.tick({.dt = 1.0, .spot_valid = false, .error = 0.0});
  EXPECT_EQ(logic.mode(), Mode::SAFE);
}

TEST(ModeLogic, SafeRecoversOnlyToIdle) {
  ModeLogic logic(params());
  logic.request(Mode::SAFE);
  EXPECT_FALSE(logic.request(Mode::ACQUIRE));
  EXPECT_EQ(logic.mode(), Mode::SAFE);
  EXPECT_TRUE(logic.request(Mode::IDLE));
  EXPECT_EQ(logic.mode(), Mode::IDLE);
}

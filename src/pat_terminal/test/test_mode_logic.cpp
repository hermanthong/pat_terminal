#include <gtest/gtest.h>
#include "pat_terminal/mode_logic.hpp"

using pat_terminal::Mode;
using pat_terminal::ModeLogic;
using pat_terminal::ModeParams;

namespace {

ModeParams params() {
  return ModeParams{
    .lock_error_threshold = 50e-6,
    .lock_debounce_s = 0.200,
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

TEST(ModeLogic, SafeRecoversOnlyToIdle) {
  ModeLogic logic(params());
  logic.request(Mode::SAFE);
  EXPECT_FALSE(logic.request(Mode::ACQUIRE));
  EXPECT_EQ(logic.mode(), Mode::SAFE);
  EXPECT_TRUE(logic.request(Mode::IDLE));
  EXPECT_EQ(logic.mode(), Mode::IDLE);
}

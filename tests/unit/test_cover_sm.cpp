/// @file test_cover_sm.cpp
/// @brief Unit tests for cover_sm — variant-based cover state machine.

#include <gtest/gtest.h>
#include "elero/cover_sm.h"
#include "elero/elero_packet.h"

namespace sm = esphome::elero::cover_sm;
namespace pkt = esphome::elero::packet;

// ═══════════════════════════════════════════════════════════════════════════════
// FIXTURES
// ═══════════════════════════════════════════════════════════════════════════════

class CoverSmTest : public ::testing::Test {
 protected:
    sm::Context ctx{
        .open_duration_ms = 10000,
        .close_duration_ms = 10000,
        .movement_timeout_ms = 120000,
        .post_stop_cooldown_ms = 3000,
    };

    sm::Context asymmetric{
        .open_duration_ms = 15000,   // Slower opening (against gravity)
        .close_duration_ms = 10000,  // Faster closing (with gravity)
        .movement_timeout_ms = 120000,
        .post_stop_cooldown_ms = 3000,
    };

    sm::Context no_tracking{
        .open_duration_ms = 0,
        .close_duration_ms = 0,
        .movement_timeout_ms = 120000,
        .post_stop_cooldown_ms = 3000,
    };

    // Tilt model: 10s wall time includes a 2s slat sweep → 8s net travel
    sm::Context tilt_ctx{
        .open_duration_ms = 10000,
        .close_duration_ms = 10000,
        .tilt_duration_ms = 2000,
        .movement_timeout_ms = 120000,
        .post_stop_cooldown_ms = 3000,
    };
};

// ═══════════════════════════════════════════════════════════════════════════════
// POSITION DERIVATION
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, PositionInterpolatesDuringOpening) {
    sm::State s = sm::Opening{0.0f, 1000};
    EXPECT_FLOAT_EQ(sm::position(s, 6000, ctx), 0.5f);
}

TEST_F(CoverSmTest, PositionInterpolatesDuringClosing) {
    sm::State s = sm::Closing{1.0f, 1000};
    EXPECT_FLOAT_EQ(sm::position(s, 6000, ctx), 0.5f);
}

TEST_F(CoverSmTest, PositionClampsAtBoundaries) {
    EXPECT_FLOAT_EQ(sm::position(sm::Opening{0.5f, 1000}, 20000, ctx), 1.0f);
    EXPECT_FLOAT_EQ(sm::position(sm::Closing{0.3f, 1000}, 20000, ctx), 0.0f);
}

TEST_F(CoverSmTest, PositionFrozenInStoppingState) {
    sm::State s = sm::Stopping{0.42f, 5000};
    EXPECT_FLOAT_EQ(sm::position(s, 5001, ctx), 0.42f);
    EXPECT_FLOAT_EQ(sm::position(s, 99999, ctx), 0.42f);
}

TEST_F(CoverSmTest, PositionStaysFixedWithoutTracking) {
    sm::State s = sm::Opening{0.3f, 1000};
    EXPECT_FLOAT_EQ(sm::position(s, 99999, no_tracking), 0.3f);
}

TEST_F(CoverSmTest, PositionUsesCorrectDurationPerDirection) {
    sm::State opening = sm::Opening{0.0f, 0};
    EXPECT_FLOAT_EQ(sm::position(opening, 7500, asymmetric), 0.5f);  // 7.5s of 15s

    sm::State closing = sm::Closing{1.0f, 0};
    EXPECT_FLOAT_EQ(sm::position(closing, 5000, asymmetric), 0.5f);  // 5s of 10s
}

TEST_F(CoverSmTest, PositionFromMidpointOpening) {
    // Opening from 0.5 for 5s of 10s total → 0.5 + 0.5 = 1.0
    sm::State s = sm::Opening{0.5f, 0};
    EXPECT_FLOAT_EQ(sm::position(s, 5000, ctx), 1.0f);
}

TEST_F(CoverSmTest, PositionFromMidpointClosing) {
    // Closing from 0.5 for 5s of 10s total → 0.5 - 0.5 = 0.0
    sm::State s = sm::Closing{0.5f, 0};
    EXPECT_FLOAT_EQ(sm::position(s, 5000, ctx), 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — DEFINITIVE ENDPOINTS (TOP / BOTTOM)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, RfTopSetsPositionOneFromAnyState) {
    auto from_idle = sm::on_rf_status(sm::Idle{0.3f}, pkt::state::TOP, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_idle));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_idle).position, 1.0f);

    auto from_opening = sm::on_rf_status(sm::Opening{0.5f, 100}, pkt::state::TOP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_opening));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_opening).position, 1.0f);

    // Closing but blind reports TOP — unexpected but authoritative
    auto from_closing = sm::on_rf_status(sm::Closing{0.8f, 100}, pkt::state::TOP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_closing));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_closing).position, 1.0f);

    // Stopping — definitive endpoint breaks through cooldown
    auto from_stopping = sm::on_rf_status(sm::Stopping{0.5f, 100}, pkt::state::TOP, 200, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_stopping));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_stopping).position, 1.0f);
}

TEST_F(CoverSmTest, RfBottomSetsPositionZeroFromAnyState) {
    auto from_idle = sm::on_rf_status(sm::Idle{0.7f}, pkt::state::BOTTOM, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_idle));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_idle).position, 0.0f);

    // Opening but blind reports BOTTOM — unexpected but authoritative
    auto from_opening = sm::on_rf_status(sm::Opening{0.5f, 100}, pkt::state::BOTTOM, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_opening));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_opening).position, 0.0f);

    auto from_closing = sm::on_rf_status(sm::Closing{0.8f, 100}, pkt::state::BOTTOM, 9000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_closing));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_closing).position, 0.0f);

    auto from_stopping = sm::on_rf_status(sm::Stopping{0.5f, 100}, pkt::state::BOTTOM, 200, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(from_stopping));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(from_stopping).position, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — MOVEMENT STARTS
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, IdleRfMovingUpStartsOpeningWithCurrentPosition) {
    auto s = sm::on_rf_status(sm::Idle{0.3f}, pkt::state::MOVING_UP, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    auto &o = std::get<sm::Opening>(s);
    EXPECT_FLOAT_EQ(o.start_position, 0.3f);
    EXPECT_EQ(o.start_ms, 100u);
}

TEST_F(CoverSmTest, IdleRfMovingDownStartsClosingWithCurrentPosition) {
    auto s = sm::on_rf_status(sm::Idle{0.7f}, pkt::state::MOVING_DOWN, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    auto &c = std::get<sm::Closing>(s);
    EXPECT_FLOAT_EQ(c.start_position, 0.7f);
    EXPECT_EQ(c.start_ms, 100u);
}

TEST_F(CoverSmTest, StartMovingVariantsBehaveIdentically) {
    auto up = sm::on_rf_status(sm::Idle{0.3f}, pkt::state::START_MOVING_UP, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(up));

    auto down = sm::on_rf_status(sm::Idle{0.7f}, pkt::state::START_MOVING_DOWN, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(down));
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — STOPPED DURING MOVEMENT (position snapshot)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, OpeningRfStoppedSnapshotsPosition) {
    auto s = sm::on_rf_status(sm::Opening{0.0f, 1000}, pkt::state::STOPPED, 6000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Stopping>(s).position, 0.5f);
}

TEST_F(CoverSmTest, ClosingRfStoppedSnapshotsPosition) {
    auto s = sm::on_rf_status(sm::Closing{1.0f, 1000}, pkt::state::STOPPED, 6000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Stopping>(s).position, 0.5f);
}

TEST_F(CoverSmTest, AllStopConditionsRouteToStopping) {
    // BLOCKING, OVERHEATED, TIMEOUT, INTERMEDIATE all go to Stopping
    auto blocking = sm::on_rf_status(sm::Opening{0.0f, 0}, pkt::state::BLOCKING, 5000, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(blocking));

    auto overheated = sm::on_rf_status(sm::Opening{0.0f, 0}, pkt::state::OVERHEATED, 5000, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(overheated));

    auto timeout = sm::on_rf_status(sm::Closing{1.0f, 0}, pkt::state::TIMEOUT, 5000, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(timeout));

    auto intermediate = sm::on_rf_status(sm::Closing{1.0f, 0}, pkt::state::INTERMEDIATE, 5000, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(intermediate));
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — DIRECTION REVERSAL (position snapshot at reversal point)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, OpeningRfMovingDownReversesWithSnapshotPosition) {
    auto s = sm::on_rf_status(sm::Opening{0.0f, 1000}, pkt::state::MOVING_DOWN, 6000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    auto &c = std::get<sm::Closing>(s);
    EXPECT_FLOAT_EQ(c.start_position, 0.5f);
    EXPECT_EQ(c.start_ms, 6000u);
}

TEST_F(CoverSmTest, ClosingRfMovingUpReversesWithSnapshotPosition) {
    auto s = sm::on_rf_status(sm::Closing{1.0f, 1000}, pkt::state::MOVING_UP, 6000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    auto &o = std::get<sm::Opening>(s);
    EXPECT_FLOAT_EQ(o.start_position, 0.5f);
    EXPECT_EQ(o.start_ms, 6000u);
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — IDEMPOTENT (same direction doesn't restart timer)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, OpeningRfMovingUpDoesNotRestartTimer) {
    sm::State s = sm::on_rf_status(sm::Opening{0.0f, 1000}, pkt::state::MOVING_UP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_EQ(std::get<sm::Opening>(s).start_ms, 1000u);
    EXPECT_FLOAT_EQ(std::get<sm::Opening>(s).start_position, 0.0f);
}

TEST_F(CoverSmTest, ClosingRfMovingDownDoesNotRestartTimer) {
    sm::State s = sm::on_rf_status(sm::Closing{1.0f, 1000}, pkt::state::MOVING_DOWN, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_EQ(std::get<sm::Closing>(s).start_ms, 1000u);
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — STOPPING COOLDOWN FILTERING
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, StoppingIgnoresAllNonEndpointRfDuringCooldown) {
    sm::State stopping = sm::Stopping{0.5f, 100};

    // Transient movement echoes from before STOP
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::MOVING_UP, 200, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::MOVING_DOWN, 200, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::START_MOVING_UP, 200, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::START_MOVING_DOWN, 200, ctx)));

    // STOPPED echo from blind confirming our STOP
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::STOPPED, 200, ctx)));

    // Safety signals also filtered during cooldown (by design — cooldown
    // is short, 3s, and we already sent STOP; re-triggering would just
    // cause noise. The blind handles its own safety internally.)
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::BLOCKING, 200, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::OVERHEATED, 200, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::TIMEOUT, 200, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_rf_status(stopping, pkt::state::INTERMEDIATE, 200, ctx)));
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF STATUS — IDLE STAYS IDLE FOR NON-MOVEMENT, NON-ENDPOINT
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, IdleIgnoresStoppedAndWarnings) {
    sm::State idle = sm::Idle{0.5f};

    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_rf_status(idle, pkt::state::STOPPED, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_rf_status(idle, pkt::state::BLOCKING, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_rf_status(idle, pkt::state::OVERHEATED, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_rf_status(idle, pkt::state::TILT, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_rf_status(idle, pkt::state::INTERMEDIATE, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_rf_status(idle, pkt::state::UNKNOWN, 100, ctx)));

    // Position unchanged
    auto s = sm::on_rf_status(idle, pkt::state::STOPPED, 100, ctx);
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).position, 0.5f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// USER COMMAND TRANSITIONS
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, IdleCmdUpOpens) {
    auto s = sm::on_command(sm::Idle{0.3f}, pkt::command::UP, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Opening>(s).start_position, 0.3f);
}

TEST_F(CoverSmTest, IdleCmdDownCloses) {
    auto s = sm::on_command(sm::Idle{0.7f}, pkt::command::DOWN, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Closing>(s).start_position, 0.7f);
}

TEST_F(CoverSmTest, OpeningCmdStopSnapshotsPosition) {
    auto s = sm::on_command(sm::Opening{0.0f, 0}, pkt::command::STOP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    EXPECT_NEAR(std::get<sm::Stopping>(s).position, 0.5f, 0.01f);
}

TEST_F(CoverSmTest, ClosingCmdStopSnapshotsPosition) {
    auto s = sm::on_command(sm::Closing{1.0f, 0}, pkt::command::STOP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    EXPECT_NEAR(std::get<sm::Stopping>(s).position, 0.5f, 0.01f);
}

TEST_F(CoverSmTest, OpeningCmdDownReversesWithSnapshotPosition) {
    auto s = sm::on_command(sm::Opening{0.0f, 0}, pkt::command::DOWN, 3000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_NEAR(std::get<sm::Closing>(s).start_position, 0.3f, 0.01f);
}

TEST_F(CoverSmTest, ClosingCmdUpReversesWithSnapshotPosition) {
    auto s = sm::on_command(sm::Closing{1.0f, 0}, pkt::command::UP, 3000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_NEAR(std::get<sm::Opening>(s).start_position, 0.7f, 0.01f);
}

TEST_F(CoverSmTest, ClosingCmdTiltReversesToOpening) {
    auto s = sm::on_command(sm::Closing{1.0f, 0}, pkt::command::TILT, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
}

TEST_F(CoverSmTest, StoppingCmdOverridesCooldownWithCorrectPosition) {
    sm::State stopping = sm::Stopping{0.5f, 100};

    auto up = sm::on_command(stopping, pkt::command::UP, 200, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(up));
    EXPECT_FLOAT_EQ(std::get<sm::Opening>(up).start_position, 0.5f);

    auto down = sm::on_command(stopping, pkt::command::DOWN, 200, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(down));
    EXPECT_FLOAT_EQ(std::get<sm::Closing>(down).start_position, 0.5f);
}

TEST_F(CoverSmTest, IntermediateCommandIsNoOpInAllStates) {
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(
        sm::on_command(sm::Idle{0.5f}, pkt::command::INTERMEDIATE, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Opening>(
        sm::on_command(sm::Opening{0.5f, 0}, pkt::command::INTERMEDIATE, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Closing>(
        sm::on_command(sm::Closing{0.5f, 0}, pkt::command::INTERMEDIATE, 100, ctx)));
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(
        sm::on_command(sm::Stopping{0.5f, 0}, pkt::command::INTERMEDIATE, 100, ctx)));
}

TEST_F(CoverSmTest, CmdUpWhileAlreadyOpeningIsNoOp) {
    sm::State orig = sm::Opening{0.0f, 1000};
    auto s = sm::on_command(orig, pkt::command::UP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    // Must not restart timer
    EXPECT_EQ(std::get<sm::Opening>(s).start_ms, 1000u);
}

TEST_F(CoverSmTest, CmdDownWhileAlreadyClosingIsNoOp) {
    sm::State orig = sm::Closing{1.0f, 1000};
    auto s = sm::on_command(orig, pkt::command::DOWN, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_EQ(std::get<sm::Closing>(s).start_ms, 1000u);
}

// ═══════════════════════════════════════════════════════════════════════════════
// TICK — TIMEOUTS AND COOLDOWN
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, OpeningTimeoutSnapshotsClampedPosition) {
    auto s = sm::on_tick(sm::Opening{0.0f, 1000}, 130000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).position, 1.0f);
}

TEST_F(CoverSmTest, ClosingTimeoutSnapshotsClampedPosition) {
    auto s = sm::on_tick(sm::Closing{1.0f, 1000}, 130000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).position, 0.0f);
}

TEST_F(CoverSmTest, StoppingCooldownExpiresPreservingPosition) {
    auto s = sm::on_tick(sm::Stopping{0.42f, 1000}, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).position, 0.42f);
}

TEST_F(CoverSmTest, TimeoutAtExactBoundary) {
    auto s = sm::on_tick(sm::Opening{0.0f, 0}, ctx.movement_timeout_ms, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(s));
}

TEST_F(CoverSmTest, CooldownAtExactBoundary) {
    auto s = sm::on_tick(sm::Stopping{0.5f, 0}, ctx.post_stop_cooldown_ms, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Idle>(s));
}

TEST_F(CoverSmTest, TimeoutOneMsBeforeBoundaryStaysMoving) {
    auto s = sm::on_tick(sm::Opening{0.0f, 0}, ctx.movement_timeout_ms - 1, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Opening>(s));
}

TEST_F(CoverSmTest, CooldownOneMsBeforeBoundaryStaysStopping) {
    auto s = sm::on_tick(sm::Stopping{0.5f, 0}, ctx.post_stop_cooldown_ms - 1, ctx);
    EXPECT_TRUE(std::holds_alternative<sm::Stopping>(s));
}

// ═══════════════════════════════════════════════════════════════════════════════
// MULTI-STEP: OPEN → STOP → RESUME (position continuity)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, OpenStopResumePreservesPositionContinuity) {
    // Open from closed
    sm::State s = sm::on_command(sm::Idle{0.0f}, pkt::command::UP, 0, ctx);

    // Stop at t=5000 (50%)
    s = sm::on_command(s, pkt::command::STOP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    EXPECT_NEAR(std::get<sm::Stopping>(s).position, 0.5f, 0.01f);

    // Resume opening
    s = sm::on_command(s, pkt::command::UP, 6000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_NEAR(std::get<sm::Opening>(s).start_position, 0.5f, 0.01f);
    EXPECT_EQ(std::get<sm::Opening>(s).start_ms, 6000u);

    // Position after 2.5s more: 0.5 + 2.5/10 = 0.75
    EXPECT_NEAR(sm::position(s, 8500, ctx), 0.75f, 0.01f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MULTI-STEP: OPEN → REVERSE → STOP (two-leg position tracking)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, OpenReverseStopTracksPositionAcrossBothLegs) {
    // Open from 0.0 at t=0
    sm::State s = sm::on_command(sm::Idle{0.0f}, pkt::command::UP, 0, ctx);

    // Reverse at t=3000 → position = 0.3
    s = sm::on_command(s, pkt::command::DOWN, 3000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_NEAR(std::get<sm::Closing>(s).start_position, 0.3f, 0.01f);

    // Stop at t=5000 → closed for 2s of 10s = 0.2 descent → 0.3 - 0.2 = 0.1
    s = sm::on_command(s, pkt::command::STOP, 5000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    EXPECT_NEAR(std::get<sm::Stopping>(s).position, 0.1f, 0.01f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MULTI-STEP: FULL CYCLE WITH RF CONFIRMATION
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, FullOpenCycleCommandThenRfConfirmation) {
    // User sends UP
    sm::State s = sm::on_command(sm::Idle{0.0f}, pkt::command::UP, 0, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));

    // Blind confirms MOVING_UP — timer NOT restarted
    s = sm::on_rf_status(s, pkt::state::MOVING_UP, 500, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_EQ(std::get<sm::Opening>(s).start_ms, 0u);

    // Blind reports TOP
    s = sm::on_rf_status(s, pkt::state::TOP, 10000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).position, 1.0f);
}

TEST_F(CoverSmTest, FullCloseCycleCommandThenRfConfirmation) {
    sm::State s = sm::on_command(sm::Idle{1.0f}, pkt::command::DOWN, 0, ctx);

    s = sm::on_rf_status(s, pkt::state::MOVING_DOWN, 500, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_EQ(std::get<sm::Closing>(s).start_ms, 0u);

    s = sm::on_rf_status(s, pkt::state::BOTTOM, 10000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).position, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MULTI-STEP: ASYMMETRIC DURATIONS
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, AsymmetricDurationsOpenReverseClose) {
    // 15s open, 10s close
    // Open for 7.5s → position = 0.5 (7.5/15)
    sm::State s = sm::on_command(sm::Idle{0.0f}, pkt::command::UP, 0, asymmetric);

    s = sm::on_command(s, pkt::command::DOWN, 7500, asymmetric);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    EXPECT_NEAR(std::get<sm::Closing>(s).start_position, 0.5f, 0.01f);

    // Close for 5s → 0.5 - 5/10 = 0.0
    s = sm::on_command(s, pkt::command::STOP, 12500, asymmetric);
    EXPECT_NEAR(std::get<sm::Stopping>(s).position, 0.0f, 0.01f);
}

TEST_F(CoverSmTest, AsymmetricDurationsOpeningRateSlowerThanClosing) {
    // After 5s: opening = 5/15 = 0.33, closing = 5/10 = 0.5
    auto open_pos = sm::position(sm::Opening{0.0f, 0}, 5000, asymmetric);
    auto close_pos = sm::position(sm::Closing{1.0f, 0}, 5000, asymmetric);

    EXPECT_NEAR(open_pos, 0.333f, 0.01f);
    EXPECT_NEAR(close_pos, 0.5f, 0.01f);
    EXPECT_GT(close_pos, open_pos);  // Closing is faster
}

// ═══════════════════════════════════════════════════════════════════════════════
// MULTI-STEP: RAPID STOP-STOP-RESUME
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, RapidStopThenResumeFromStoppingState) {
    // Opening
    sm::State s = sm::on_command(sm::Idle{0.0f}, pkt::command::UP, 0, ctx);

    // Stop at t=3000 (position 0.3)
    s = sm::on_command(s, pkt::command::STOP, 3000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));

    // Another STOP during cooldown — should stay Stopping
    s = sm::on_command(s, pkt::command::STOP, 3500, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));

    // Resume UP during cooldown — overrides
    s = sm::on_command(s, pkt::command::UP, 4000, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_NEAR(std::get<sm::Opening>(s).start_position, 0.3f, 0.01f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// TILT MODEL — tilt-before-lift dead time
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(CoverSmTest, HasTiltTrackingRequiresDuration) {
    EXPECT_FALSE(sm::has_tilt_tracking(ctx));
    EXPECT_TRUE(sm::has_tilt_tracking(tilt_ctx));
}

TEST_F(CoverSmTest, TiltFrozenWhileOpeningFromSlatsClosed) {
    // Opening from position 0.5, slats closed: dead time = (1-0) * 2000ms
    sm::State s = sm::Opening{0.5f, 0, 0.0f};
    EXPECT_FLOAT_EQ(sm::position(s, 1000, tilt_ctx), 0.5f);  // mid-tilt: frozen
    EXPECT_FLOAT_EQ(sm::tilt(s, 1000, tilt_ctx), 0.5f);     // 1000/2000 sweep
    EXPECT_FLOAT_EQ(sm::position(s, 1999, tilt_ctx), 0.5f);  // still frozen
    EXPECT_FLOAT_EQ(sm::tilt(s, 2000, tilt_ctx), 1.0f);      // slats open
}

TEST_F(CoverSmTest, TiltFrozenWhileClosingFromSlatsOpen) {
    // Closing from position 0.5, slats open: dead time = 1.0 * 2000ms
    sm::State s = sm::Closing{0.5f, 0, 1.0f};
    EXPECT_FLOAT_EQ(sm::position(s, 1000, tilt_ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::tilt(s, 1000, tilt_ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::tilt(s, 2000, tilt_ctx), 0.0f);
}

TEST_F(CoverSmTest, PositionMovesImmediatelyWhenSlatsAlreadyOpen) {
    // Opening at 50% with slats already open — zero dead time
    sm::State s = sm::Opening{0.5f, 0, 1.0f};
    EXPECT_FLOAT_EQ(sm::position(s, 2000, tilt_ctx), 0.75f);  // 2s of 8s net
}

TEST_F(CoverSmTest, WallTimeDurationMatchesFullRunFromClosed) {
    // Full open from (pos 0, slats closed) must take exactly open_duration wall time
    sm::State s = sm::Opening{0.0f, 0, 0.0f};
    EXPECT_FLOAT_EQ(sm::position(s, 9999, tilt_ctx), 0.999875f);  // 7999/8000
    EXPECT_FLOAT_EQ(sm::position(s, 10000, tilt_ctx), 1.0f);
}

TEST_F(CoverSmTest, TiltAdvancesLinearlyDuringSweep) {
    sm::State s = sm::Opening{0.0f, 0, 0.25f};
    EXPECT_FLOAT_EQ(sm::tilt(s, 500, tilt_ctx), 0.5f);   // 0.25 + 500/2000
    EXPECT_FLOAT_EQ(sm::tilt(s, 1500, tilt_ctx), 1.0f);  // clamped
}

TEST_F(CoverSmTest, ReversalIncludesFullSweepBackDeadTime) {
    // Opening for 4000ms from (0, slats closed): travel 2000/8000 → pos 0.25
    sm::State s = sm::Opening{0.0f, 0, 0.0f};
    ASSERT_NEAR(sm::position(s, 4000, tilt_ctx), 0.25f, 0.001f);

    // Reverse: slats must sweep all the way back (start_tilt = 1.0 at reversal)
    s = sm::on_command(s, pkt::command::DOWN, 4000, tilt_ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Closing>(s));
    auto &c = std::get<sm::Closing>(s);
    EXPECT_FLOAT_EQ(c.start_tilt, 1.0f);
    EXPECT_NEAR(c.start_position, 0.25f, 0.001f);

    // During the 2s re-tilt phase position stays frozen at 0.25
    EXPECT_NEAR(sm::position(s, 6000, tilt_ctx), 0.25f, 0.001f);
    // ...then travels: at t=10s total, 4s of net travel from 0.25 → 0.25 - 4/8 = -0.25 → clamped 0
    EXPECT_NEAR(sm::position(s, 10000, tilt_ctx), 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(sm::tilt(s, 10000, tilt_ctx), 0.0f);
}

TEST_F(CoverSmTest, StopMidTiltFreezesBothPositionAndTilt) {
    sm::State s = sm::Opening{0.5f, 0, 0.0f};
    s = sm::on_command(s, pkt::command::STOP, 1000, tilt_ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Stopping>(s));
    auto &st = std::get<sm::Stopping>(s);
    EXPECT_FLOAT_EQ(st.position, 0.5f);
    EXPECT_FLOAT_EQ(st.tilt, 0.5f);

    // Derived values frozen while cooling down
    EXPECT_FLOAT_EQ(sm::position(s, 2000, tilt_ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::tilt(s, 2000, tilt_ctx), 0.5f);

    // Cooldown expiry preserves both
    s = sm::on_tick(s, 4000, tilt_ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(s).tilt, 0.5f);

    // Resuming from a partial tilt: dead time only covers the remaining sweep
    s = sm::on_command(s, pkt::command::UP, 4000, tilt_ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Opening>(s).start_tilt, 0.5f);
    // 1000ms of tilt sweep remain before travel begins
    EXPECT_FLOAT_EQ(sm::position(s, 4999, tilt_ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::position(s, 5000, tilt_ctx), 0.5f);
    EXPECT_NEAR(sm::position(s, 5800, tilt_ctx), 0.6f, 0.001f);  // 800ms of 8s net
}

TEST_F(CoverSmTest, TiltDefaultsToDirectionExtremesWithoutDuration) {
    // No tilt_duration → movement reports direction extremes (legacy binary behavior)
    sm::State opening = sm::Opening{0.0f, 0};
    EXPECT_FLOAT_EQ(sm::tilt(opening, 5000, ctx), 1.0f);
    EXPECT_FLOAT_EQ(sm::tilt(sm::Closing{1.0f, 0}, 5000, ctx), 0.0f);
}

TEST_F(CoverSmTest, RfEndpointsSetTilt) {
    auto top = sm::on_rf_status(sm::Idle{0.3f, 0.5f}, pkt::state::TOP, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(top));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(top).position, 1.0f);
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(top).tilt, 0.0f);

    auto top_tilt = sm::on_rf_status(sm::Idle{0.3f, 0.5f}, pkt::state::TOP_TILT, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(top_tilt));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(top_tilt).position, 1.0f);
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(top_tilt).tilt, 1.0f);

    // TILT byte while idle: at stored tilt favorite — position guess preserved
    auto tilt = sm::on_rf_status(sm::Idle{0.4f, 0.0f}, pkt::state::TILT, 100, ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Idle>(tilt));
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(tilt).position, 0.4f);
    EXPECT_FLOAT_EQ(std::get<sm::Idle>(tilt).tilt, 1.0f);
}

TEST_F(CoverSmTest, RfMovementFromIdlePreservesTilt) {
    auto s = sm::on_rf_status(sm::Idle{0.5f, 0.7f}, pkt::state::MOVING_UP, 100, tilt_ctx);
    ASSERT_TRUE(std::holds_alternative<sm::Opening>(s));
    EXPECT_FLOAT_EQ(std::get<sm::Opening>(s).start_tilt, 0.7f);
    // Dead time covers only the remaining 30% of the sweep: 0.3 * 2000ms = 600ms
    EXPECT_FLOAT_EQ(sm::position(s, 100, tilt_ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::position(s, 700, tilt_ctx), 0.5f);
    // At t=1600: (1600-100-600)/8000 = 900ms net → 0.5 + 900/8000 = 0.6125
    EXPECT_NEAR(sm::position(s, 1600, tilt_ctx), 0.6125f, 0.001f);
}

TEST_F(CoverSmTest, TiltDurationZeroKeepsLegacyPositionMath) {
    // With tilt_duration_ms = 0 the start_tilt anchor is ignored and the
    // position derivation is byte-identical to the pre-tilt model.
    sm::State legacy = sm::Opening{0.0f, 1000};
    sm::State with_tilt_field = sm::Opening{0.0f, 1000, 0.7f};

    EXPECT_FLOAT_EQ(sm::position(legacy, 6000, ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::position(with_tilt_field, 6000, ctx), 0.5f);
    EXPECT_FLOAT_EQ(sm::position(with_tilt_field, 20000, ctx), 1.0f);
}

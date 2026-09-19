/// @file cover_sm.cpp
/// @brief Cover state machine transition implementations.

#include "cover_sm.h"
#include "overloaded.h"
#include "elero_packet.h"
#include <algorithm>

namespace esphome::elero::cover_sm {

// ─── Position/tilt helpers ──────────────────────────────────────────────────

/// Net travel time with the tilt phase removed. Wall-time durations include
/// one full tilt sweep (measured from fully closed); guard against bad config
/// (tilt_duration >= travel duration) by clamping to a 1 ms minimum.
static float net_travel_ms(uint32_t dur, uint32_t tilt_dur) {
    return static_cast<float>(dur > tilt_dur ? dur - tilt_dur : 1u);
}

/// Tilt-before-lift dead time in ms: the remaining slat sweep in the movement
/// direction. Opening sweeps toward TILT_OPEN, closing toward TILT_CLOSED.
static float tilt_dead_time(float start_tilt, uint32_t tilt_dur, bool opening) {
    if (tilt_dur == 0) return 0.0f;
    float headroom = opening ? (TILT_OPEN - start_tilt) : (start_tilt - TILT_CLOSED);
    if (headroom < 0.0f) headroom = 0.0f;
    return headroom * static_cast<float>(tilt_dur);
}

static float position_during_opening(float start, float start_tilt,
                                     uint32_t start_ms, uint32_t now,
                                     uint32_t dur, uint32_t tilt_dur) {
    if (dur == 0) return start;
    float travel_elapsed = static_cast<float>(now - start_ms)
                         - tilt_dead_time(start_tilt, tilt_dur, true);
    if (travel_elapsed < 0.0f) return start;  // still tilting — position frozen
    return std::min(POSITION_OPEN, start + travel_elapsed / net_travel_ms(dur, tilt_dur));
}

static float position_during_closing(float start, float start_tilt,
                                     uint32_t start_ms, uint32_t now,
                                     uint32_t dur, uint32_t tilt_dur) {
    if (dur == 0) return start;
    float travel_elapsed = static_cast<float>(now - start_ms)
                         - tilt_dead_time(start_tilt, tilt_dur, false);
    if (travel_elapsed < 0.0f) return start;  // still tilting — position frozen
    return std::max(POSITION_CLOSED, start - travel_elapsed / net_travel_ms(dur, tilt_dur));
}

static float tilt_during_opening(float start_tilt, uint32_t start_ms,
                                 uint32_t now, uint32_t tilt_dur) {
    if (tilt_dur == 0) return TILT_OPEN;
    float elapsed = static_cast<float>(now - start_ms) / static_cast<float>(tilt_dur);
    return std::min(TILT_OPEN, start_tilt + elapsed);
}

static float tilt_during_closing(float start_tilt, uint32_t start_ms,
                                 uint32_t now, uint32_t tilt_dur) {
    if (tilt_dur == 0) return TILT_CLOSED;
    float elapsed = static_cast<float>(now - start_ms) / static_cast<float>(tilt_dur);
    return std::max(TILT_CLOSED, start_tilt - elapsed);
}

// ─── Derived values ─────────────────────────────────────────────────────────

float position(const State &state, uint32_t now, const Context &ctx) {
    return std::visit(overloaded{
        [](const Idle &s) -> float { return s.position; },
        [](const Stopping &s) -> float { return s.position; },
        [&](const Opening &s) -> float {
            return position_during_opening(s.start_position, s.start_tilt,
                                           s.start_ms, now,
                                           ctx.open_duration_ms,
                                           ctx.tilt_duration_ms);
        },
        [&](const Closing &s) -> float {
            return position_during_closing(s.start_position, s.start_tilt,
                                           s.start_ms, now,
                                           ctx.close_duration_ms,
                                           ctx.tilt_duration_ms);
        },
    }, state);
}

float tilt(const State &state, uint32_t now, const Context &ctx) {
    return std::visit(overloaded{
        [](const Idle &s) -> float { return s.tilt; },
        [](const Stopping &s) -> float { return s.tilt; },
        [&](const Opening &s) -> float {
            return tilt_during_opening(s.start_tilt, s.start_ms, now,
                                       ctx.tilt_duration_ms);
        },
        [&](const Closing &s) -> float {
            return tilt_during_closing(s.start_tilt, s.start_ms, now,
                                       ctx.tilt_duration_ms);
        },
    }, state);
}

bool has_position_tracking(const Context &ctx) {
    return ctx.open_duration_ms > 0 && ctx.close_duration_ms > 0;
}

bool has_tilt_tracking(const Context &ctx) {
    return ctx.tilt_duration_ms > 0;
}

bool is_moving(const State &state) {
    return std::holds_alternative<Opening>(state) ||
           std::holds_alternative<Closing>(state);
}

bool is_idle(const State &state) {
    return std::holds_alternative<Idle>(state) ||
           std::holds_alternative<Stopping>(state);
}

Operation operation(const State &state) {
    return std::visit(overloaded{
        [](const Idle &) { return Operation::IDLE; },
        [](const Stopping &) { return Operation::IDLE; },
        [](const Opening &) { return Operation::OPENING; },
        [](const Closing &) { return Operation::CLOSING; },
    }, state);
}

// ─── Helpers for transitions ────────────────────────────────────────────────

/// Classify an RF state byte as indicating upward movement.
static bool is_rf_moving_up(uint8_t s) {
    return s == packet::state::MOVING_UP || s == packet::state::START_MOVING_UP;
}

/// Classify an RF state byte as indicating downward movement.
static bool is_rf_moving_down(uint8_t s) {
    return s == packet::state::MOVING_DOWN || s == packet::state::START_MOVING_DOWN;
}

/// Classify an RF state byte as indicating movement stopped (any reason).
static bool is_rf_stopped(uint8_t s) {
    return s == packet::state::STOPPED ||
           s == packet::state::INTERMEDIATE ||
           s == packet::state::BLOCKING ||
           s == packet::state::OVERHEATED ||
           s == packet::state::TIMEOUT;
}

// ─── RF Status transitions ─────────────────────────────────────────────────

State on_rf_status(const State &state, uint8_t state_byte, uint32_t now,
                   const Context &ctx) {
    const uint8_t s = state_byte;
    // Tilt reported via RF status bytes: TOP/BOTTOM = not tilted (slats open),
    // *_TILT = tilted. TILT alone means "at stored tilt position".
    const float rf_tilt = (s == packet::state::TOP_TILT ||
                           s == packet::state::BOTTOM_TILT ||
                           s == packet::state::TILT) ? TILT_OPEN : TILT_CLOSED;

    return std::visit(overloaded{
        // ── From Idle ──────────────────────────────────────────────
        [&](const Idle &idle) -> State {
            if (s == packet::state::TOP) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::BOTTOM) return Idle{POSITION_CLOSED, rf_tilt};
            if (s == packet::state::TOP_TILT) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::BOTTOM_TILT) return Idle{POSITION_CLOSED, rf_tilt};
            // Blind stopped at its stored tilt position — keep position guess
            if (s == packet::state::TILT) return Idle{idle.position, rf_tilt};
            if (is_rf_moving_up(s)) return Opening{idle.position, now, idle.tilt};
            if (is_rf_moving_down(s)) return Closing{idle.position, now, idle.tilt};
            // INTERMEDIATE, warnings — stay idle
            return state;
        },

        // ── From Opening ───────────────────────────────────────────
        [&](const Opening &) -> State {
            if (s == packet::state::TOP) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::BOTTOM) return Idle{POSITION_CLOSED, rf_tilt};
            if (s == packet::state::TOP_TILT) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::BOTTOM_TILT) return Idle{POSITION_CLOSED, rf_tilt};
            if (s == packet::state::TILT) {
                float pos = position(state, now, ctx);
                return Stopping{pos, now, rf_tilt};
            }
            if (is_rf_stopped(s)) {
                float pos = position(state, now, ctx);
                float t = tilt(state, now, ctx);
                return Stopping{pos, now, t};
            }
            // Direction change from blind itself
            if (is_rf_moving_down(s)) {
                float pos = position(state, now, ctx);
                float t = tilt(state, now, ctx);
                return Closing{pos, now, t};
            }
            // Still opening — no change
            return state;
        },

        // ── From Closing ───────────────────────────────────────────
        [&](const Closing &) -> State {
            if (s == packet::state::BOTTOM) return Idle{POSITION_CLOSED, rf_tilt};
            if (s == packet::state::TOP) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::TOP_TILT) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::BOTTOM_TILT) return Idle{POSITION_CLOSED, rf_tilt};
            if (s == packet::state::TILT) {
                float pos = position(state, now, ctx);
                return Stopping{pos, now, rf_tilt};
            }
            if (is_rf_stopped(s)) {
                float pos = position(state, now, ctx);
                float t = tilt(state, now, ctx);
                return Stopping{pos, now, t};
            }
            // Direction change from blind itself
            if (is_rf_moving_up(s)) {
                float pos = position(state, now, ctx);
                float t = tilt(state, now, ctx);
                return Opening{pos, now, t};
            }
            // Still closing — no change
            return state;
        },

        // ── From Stopping (cooldown) ───────────────────────────────
        [&](const Stopping &) -> State {
            // During cooldown, only definitive endpoint states are respected.
            // Transient MOVING_UP/DOWN are ignored (they're echoes from before STOP).
            if (s == packet::state::TOP) return Idle{POSITION_OPEN, rf_tilt};
            if (s == packet::state::BOTTOM) return Idle{POSITION_CLOSED, rf_tilt};
            return state;
        },
    }, state);
}

// ─── User Command transitions ──────────────────────────────────────────────

State on_command(const State &state, uint8_t cmd_byte, uint32_t now,
                 const Context &ctx) {
    const uint8_t cmd = cmd_byte;
    float pos = position(state, now, ctx);
    float t = tilt(state, now, ctx);

    return std::visit(overloaded{
        [&](const Idle &) -> State {
            if (cmd == packet::command::UP || cmd == packet::command::TILT)
                return Opening{pos, now, t};
            if (cmd == packet::command::DOWN)
                return Closing{pos, now, t};
            return state;  // CHECK, STOP while idle — no change
        },

        [&](const Opening &) -> State {
            if (cmd == packet::command::STOP)
                return Stopping{pos, now, t};
            if (cmd == packet::command::DOWN)
                return Closing{pos, now, t};  // Reverse — full tilt sweep back
            return state;
        },

        [&](const Closing &) -> State {
            if (cmd == packet::command::STOP)
                return Stopping{pos, now, t};
            if (cmd == packet::command::UP || cmd == packet::command::TILT)
                return Opening{pos, now, t};  // Reverse — full tilt sweep back
            return state;
        },

        [&](const Stopping &) -> State {
            // User commands override cooldown
            if (cmd == packet::command::UP || cmd == packet::command::TILT)
                return Opening{pos, now, t};
            if (cmd == packet::command::DOWN)
                return Closing{pos, now, t};
            return state;
        },
    }, state);
}

// ─── Tick transitions (timeouts) ────────────────────────────────────────────

State on_tick(const State &state, uint32_t now, const Context &ctx) {
    return std::visit(overloaded{
        [&](const Idle &) -> State { return state; },

        [&](const Opening &s) -> State {
            if ((now - s.start_ms) >= ctx.movement_timeout_ms) {
                return Idle{position(state, now, ctx), tilt(state, now, ctx)};
            }
            return state;
        },

        [&](const Closing &s) -> State {
            if ((now - s.start_ms) >= ctx.movement_timeout_ms) {
                return Idle{position(state, now, ctx), tilt(state, now, ctx)};
            }
            return state;
        },

        [&](const Stopping &s) -> State {
            if ((now - s.stop_ms) >= ctx.post_stop_cooldown_ms) {
                return Idle{s.position, s.tilt};
            }
            return state;
        },
    }, state);
}

}  // namespace esphome::elero::cover_sm

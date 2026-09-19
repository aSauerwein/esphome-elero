/// @file cover_sm.h
/// @brief Cover state machine — variant-based FSM with derived position and tilt.
///
/// States carry exactly the data needed for that state.
/// Position AND tilt are always DERIVED from (state, now, durations), never
/// stored separately. Invalid transitions don't compile — std::visit forces
/// exhaustive handling.
///
/// Tilt model (venetian "slat tilt before lift"):
/// The first phase of any directional move only rotates the slats; the blind
/// itself does not travel. The tilt dead-time depends on the current tilt and
/// the movement direction: opening sweeps tilt toward TILT_OPEN, closing
/// toward TILT_CLOSED. Position advances only from the time that remains
/// after the tilt phase. This matches the embedded-phase model used by
/// Tasmota (ShutterTiltConfig), bruxy70/Venetian-Blinds-Control and Shelly
/// slats. open/close_duration_ms are WALL-TIME durations measured from fully
/// closed (+ slats closed) to fully open, i.e. they include one tilt sweep;
/// net travel time = duration - tilt_duration.
///
/// States: Idle, Opening, Closing, Stopping
/// Stopping = post-stop cooldown (replaces the old idle_from_stop boolean)

#pragma once

#include <cstdint>
#include <variant>

namespace esphome::elero::cover_sm {

// ═══════════════════════════════════════════════════════════════════════════════
// POSITION CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

constexpr float POSITION_CLOSED = 0.0f;   ///< Fully closed (bottom)
constexpr float POSITION_OPEN = 1.0f;     ///< Fully open (top)
constexpr float NO_TARGET = -1.0f;        ///< No intermediate target position

// ═══════════════════════════════════════════════════════════════════════════════
// TILT CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

constexpr float TILT_CLOSED = 0.0f;       ///< Slats fully closed (HA tilt 0%)
constexpr float TILT_OPEN = 1.0f;         ///< Slats fully open (HA tilt 100%)
constexpr float TILT_UNKNOWN = 0.5f;      ///< Unknown at boot (mirrors position)

// ═══════════════════════════════════════════════════════════════════════════════
// STATES — each carries exactly its own data
// ═══════════════════════════════════════════════════════════════════════════════

struct Idle {
    float position{0.5f};        ///< Position (0=closed, 1=open, 0.5=unknown at boot)
    float tilt{TILT_UNKNOWN};    ///< Tilt (0=slats closed, 1=slats open, 0.5=unknown)
};

struct Opening {
    float    start_position{POSITION_CLOSED};
    uint32_t start_ms{0};
    float    start_tilt{TILT_CLOSED};  ///< Tilt at movement start (dead-time anchor)
};

struct Closing {
    float    start_position{POSITION_CLOSED};
    uint32_t start_ms{0};
    float    start_tilt{TILT_OPEN};    ///< Tilt at movement start (dead-time anchor)
};

/// Post-stop cooldown state. Transient RF "still moving" responses are
/// blocked for post_stop_cooldown_ms after an explicit STOP.
/// User commands override the cooldown.
struct Stopping {
    float    position{POSITION_CLOSED};  ///< Frozen at moment of stop
    uint32_t stop_ms{0};
    float    tilt{TILT_UNKNOWN};         ///< Frozen at moment of stop
};

using State = std::variant<Idle, Opening, Closing, Stopping>;

// ═══════════════════════════════════════════════════════════════════════════════
// CONTEXT — immutable device parameters, passed to transition functions
// ═══════════════════════════════════════════════════════════════════════════════

struct Context {
    uint32_t open_duration_ms{0};       ///< Wall time fully closed→open, incl. tilt sweep
    uint32_t close_duration_ms{0};      ///< Wall time fully open→closed, incl. tilt sweep
    uint32_t tilt_duration_ms{0};       ///< Time for full slat sweep (0 = no tilt model)
    uint32_t movement_timeout_ms{120000};     ///< Force idle after this long
    uint32_t post_stop_cooldown_ms{3000};     ///< Ignore transient RF after STOP
};

// ═══════════════════════════════════════════════════════════════════════════════
// DERIVED VALUES — pure functions, no mutation
// ═══════════════════════════════════════════════════════════════════════════════

/// Current cover position (0.0 = closed, 1.0 = open).
/// Computed from state + elapsed time. Never stored.
float position(const State &state, uint32_t now, const Context &ctx);

/// Current tilt (0.0 = slats closed, 1.0 = slats open).
/// Computed from state + elapsed time. Never stored.
float tilt(const State &state, uint32_t now, const Context &ctx);

/// Whether the cover supports position tracking (both durations non-zero).
bool has_position_tracking(const Context &ctx);

/// Whether the tilt model is active (tilt_duration > 0).
/// Without it, tilt is reported as movement-direction extremes during moves
/// and as the last RF-reported value while idle (legacy binary behavior).
bool has_tilt_tracking(const Context &ctx);

/// Whether the cover is currently in motion.
bool is_moving(const State &state);

/// Whether the cover is in a stable state (Idle or Stopping).
bool is_idle(const State &state);

/// ESPHome-compatible operation enum.
enum class Operation : uint8_t { IDLE = 0, OPENING = 1, CLOSING = 2 };

/// Map current state to ESPHome operation enum.
Operation operation(const State &state);

// ═══════════════════════════════════════════════════════════════════════════════
// TRANSITIONS — pure functions returning new state
// ═══════════════════════════════════════════════════════════════════════════════

/// Process an RF status packet from the blind.
/// @param state Current state
/// @param state_byte The Elero state byte (packet::state::*)
/// @param now Current millis()
/// @param ctx Device parameters
/// @return New state (may be unchanged)
State on_rf_status(const State &state, uint8_t state_byte, uint32_t now, const Context &ctx);

/// Process a user command (from HA, web UI, MQTT, Matter, etc.).
/// @param state Current state
/// @param cmd_byte The Elero command byte (packet::command::*)
/// @param now Current millis()
/// @param ctx Device parameters
/// @return New state (may be unchanged)
State on_command(const State &state, uint8_t cmd_byte, uint32_t now, const Context &ctx);

/// Process a time tick — checks movement timeout and cooldown expiration.
/// Call this periodically from loop().
/// @return New state (may be unchanged)
State on_tick(const State &state, uint32_t now, const Context &ctx);

}  // namespace esphome::elero::cover_sm

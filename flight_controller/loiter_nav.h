/*************************************************************************************************************
 * Loiter: a fixed-bank orbit flown by the flight controller.
 *
 * Loiter v1 performs NO navigation. It substitutes a constant desired bank and
 * a level desired pitch for the pilot's stick inside the existing Fly-By-Wire
 * branch, and the same attitude PIDs that fly a hand-commanded bank fly the
 * orbit. The aircraft therefore circles wherever it happens to be and drifts
 * downwind; holding the circle over a fixed point needs a position loop, which
 * is deliberately a later step.
 *
 * The mode lives here rather than on the ground station because the ground
 * station is exactly the wrong place for it: a pilot reaches for loiter when
 * the model is far away and they need a moment, which is when the link is
 * weakest. Closing this loop on the FC means the orbit is immune to ground
 * station stalls, telemetry jitter, and the sub-failsafe dropouts that would
 * otherwise glitch a command stream. It is also the only structure that a
 * future return-to-home can build on.
 *
 * What this does NOT change is the RC failsafe. On RC loss the firmware still
 * forces Manual and cuts throttle, and the caller drops loiter with it, so the
 * model glides exactly as it does today. Surviving a link loss under power is
 * a separate and far more dangerous change than this one.
 *
 * Everything here is pure predicate and float math -- no Arduino, matrix
 * library, or konfig dependencies -- so the exact flight code is compiled and
 * verified on the host by tests/loiter_test.cpp.
 *
 * Angle conventions match quaternionToEulerDeg in Main.ino, the same ones the
 * FBW PIDs already compare against.
 ************************************************************************************************************/
#ifndef LOITER_NAV_H
#define LOITER_NAV_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * CH10 / AUX6 request band
 *
 * Mirrors the CH6 control-mode and CH7 throttle-mode encoding: an explicit
 * high value requests the mode and every other value means off, so a centered,
 * failsafed, or transient channel can never latch the aircraft into an orbit.
 * CH8/CH9 carry the board-alignment trim, which is why this is CH10.
 * ------------------------------------------------------------------------- */
#define LOITER_REQUEST_TARGET   1700
#define LOITER_REQUEST_DEADBAND 150
#define LOITER_REQUEST_MIN      (LOITER_REQUEST_TARGET - LOITER_REQUEST_DEADBAND) /* 1550 */

/* ---------------------------------------------------------------------------
 * Orbit geometry
 *
 * The bank is well inside the FBW hard limit so that limit stays a redundant
 * backstop rather than an active clamp. Level pitch: v1 holds no altitude, the
 * bank does the work and the airframe's trim decides whether it sinks.
 *
 * LOITER_BANK_DIRECTION follows the firmware's roll convention (left roll
 * positive, see docs/protocol_contract.md), NOT a compass direction. Which way
 * the model actually circles is a first-flight observation; flip the sign here
 * and reflash if it turns the wrong way.
 * ------------------------------------------------------------------------- */
#define LOITER_BANK_ANGLE_DEG  20.0f
#define LOITER_PITCH_ANGLE_DEG 0.0f
#define LOITER_BANK_DIRECTION  1.0f

/* True when CH10 explicitly requests the orbit. */
static inline bool loiterRequestedFromChannel(uint16_t channelValue)
{
    return channelValue >= (uint16_t)LOITER_REQUEST_MIN;
}

/* Whether the orbit may run this cycle.
 *
 * Every condition is required continuously, not just at entry, so the caller
 * can evaluate this every control cycle and the orbit falls out the moment any
 * of them stops holding:
 *
 *   requested       - CH10 is driven high by the ground station.
 *   rcFresh         - the link is alive. On RC loss the caller's failsafe
 *                     already forces Manual and cuts throttle; loiter must not
 *                     outlive that.
 *   fbwActive       - loiter substitutes a setpoint for the FBW PIDs, so it is
 *                     meaningless unless those PIDs are the ones flying.
 *   attitudeUsable  - the attitude estimate is fresh AND converged. Banking on
 *                     a frozen or still-settling estimate would hold whatever
 *                     error the filter last believed.
 *   airborne        - the latched airborne state. A commanded 20 degree bank
 *                     during a ground roll is a dropped wingtip.
 */
static inline bool loiterMayEngage(bool requested,
                                   bool rcFresh,
                                   bool fbwActive,
                                   bool attitudeUsable,
                                   bool airborne)
{
    return requested && rcFresh && fbwActive && attitudeUsable && airborne;
}

/* Desired attitude for the orbit, in the FBW PIDs' own convention.
 *
 * Clamped into the caller's FBW envelope so that editing the bank constant
 * above can never command past the limit the rest of the firmware enforces.
 */
static inline void loiterDesiredAttitude(float maxRollDeg,
                                         float maxPitchDeg,
                                         float* desiredRollDeg,
                                         float* desiredPitchDeg)
{
    float roll = LOITER_BANK_ANGLE_DEG * LOITER_BANK_DIRECTION;
    float pitch = LOITER_PITCH_ANGLE_DEG;

    const float rollLimit = (maxRollDeg < 0.0f) ? -maxRollDeg : maxRollDeg;
    const float pitchLimit = (maxPitchDeg < 0.0f) ? -maxPitchDeg : maxPitchDeg;

    if (roll > rollLimit) { roll = rollLimit; }
    if (roll < -rollLimit) { roll = -rollLimit; }
    if (pitch > pitchLimit) { pitch = pitchLimit; }
    if (pitch < -pitchLimit) { pitch = -pitchLimit; }

    if (desiredRollDeg != 0) { *desiredRollDeg = roll; }
    if (desiredPitchDeg != 0) { *desiredPitchDeg = pitch; }
}

#endif /* LOITER_NAV_H */

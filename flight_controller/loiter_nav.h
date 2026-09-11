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
#define LOITER_BANK_ANGLE_DEG  15.0f
#define LOITER_PITCH_ANGLE_DEG 0.0f
#define LOITER_BANK_DIRECTION  1.0f

/* ---------------------------------------------------------------------------
 * Altitude hold
 *
 * A fixed bank with a level PITCH ATTITUDE does not hold height: in a turn the
 * lift vector tilts, so at 15 deg of bank about 3.4% of it no longer opposes
 * gravity and the orbit settles the whole time it runs. Holding altitude turns
 * the orbit from something the pilot must watch descend into something that
 * stays put.
 *
 * The loop is deliberately timid, because pitch-holds-altitude/throttle-holds-
 * speed is the coupling that kills aeroplanes:
 *
 *   - PROPORTIONAL ONLY. An integrator against an airframe whose phugoid
 *     period has not been measured will wind up and porpoise. Steady-state
 *     droop is the accepted price: expect to settle a few metres low, because
 *     holding height in the bank needs a small standing nose-up command and
 *     only a standing error can produce one without an integrator.
 *   - A hard pitch clamp well inside the FBW envelope, so the loop can never
 *     command a large attitude however large the altitude error grows.
 *   - An AIRSPEED FLOOR that outranks altitude entirely. A loop that pitches
 *     up to hold height while the throttle cannot sustain the climb will fly
 *     the wing to a stall. Below the floor -- or with no trustworthy airspeed
 *     at all -- the altitude loop is abandoned and pitch returns to level,
 *     which is the un-held orbit: it descends and gains speed. That is the
 *     same behaviour as having no altitude hold, so the degraded case is never
 *     worse than not having the feature.
 *
 * LOITER_MIN_AIRSPEED_MPH MUST be set above the airframe's measured clean
 * stall speed before flying this. The shipped value is a placeholder chosen
 * below the auto-throttle default, not a measurement.
 * ------------------------------------------------------------------------- */
#define LOITER_ALT_KP_DEG_PER_M    0.5f
#define LOITER_ALT_PITCH_LIMIT_DEG 10.0f
#define LOITER_MIN_AIRSPEED_MPH    18.0f

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
 *   airborne        - the latched airborne state AND the caller's confidence
 *                     that the latch still means anything. A commanded bank
 *                     during a ground roll is a dropped wingtip, so this must
 *                     never be the raw flag.
 *
 *                     The firmware's latch clears on HEIGHT, and height comes
 *                     from a barometer reading that a failed sensor leaves
 *                     frozen -- which pins the latch set and hides the landing
 *                     completely, leaving an orbit commanding its bank on the
 *                     runway. Callers must therefore fold in the freshness of
 *                     EVERY sensor their own latch depends on to clear, not
 *                     just the ones it depends on to set. Main.ino passes
 *                     `aircraftAirborne && barometerInputFresh(...)`.
 */
static inline bool loiterMayEngage(bool requested,
                                   bool rcFresh,
                                   bool fbwActive,
                                   bool attitudeUsable,
                                   bool airborne)
{
    return requested && rcFresh && fbwActive && attitudeUsable && airborne;
}

/* Latched loiter state.
 *
 * loiterMayEngage() alone is not enough to drive the orbit, because it is a
 * CONTINUOUS predicate: a request standing on CH10 while the aircraft is still
 * on the ground would satisfy it the instant the airborne latch set, and the
 * model would roll into the orbit moments after takeoff without the operator
 * touching anything. Requiring a rising edge on the request closes
 * that: a request that was not flyable when it arrived stays refused until the
 * ground station drops CH10 and raises it again.
 */
typedef struct {
    bool running;        /* the orbit is flying this cycle */
    bool prevRequested;  /* CH10 state last cycle, for edge detection */
    bool transitioned;   /* running changed on this call */
    /* Altitude captured when this orbit engaged, and whether it is usable.
     * Taken at engage rather than from a ground reference so the hold works
     * from wherever the pilot chose to start circling, and needs no agreement
     * with the ground station about what the target is. */
    float targetAltitudeM;
    bool  targetAltitudeValid;
} LoiterState;

/* Initialise the latch. Boot-time only: see loiterUpdate() for why calling
 * this from the RC failsafe would reintroduce automatic loiter resume. */
static inline void loiterStateInit(LoiterState* st)
{
    st->running = false;
    st->prevRequested = false;
    st->transitioned = false;
    st->targetAltitudeM = 0.0f;
    st->targetAltitudeValid = false;
}

/* Advance the latch and return whether the orbit flies this cycle.
 *
 * Call this EVERY control cycle, before the caller picks a servo mode, and
 * pass the real gate values. Calling it only from inside a Fly-By-Wire branch
 * hides failing gates that route control elsewhere: an unusable attitude
 * estimate takes a pass-through path, the latch never observes it, and the
 * orbit resumes silently once the estimate recovers.
 *
 * Do NOT reset the state from the RC failsafe. It is tempting -- the orbit
 * must not outlive the link -- but the rcFresh gate already stops it, and
 * clearing the latch makes a standing high CH10 look released, so the first
 * recovered packet reads as a synthetic rising edge and the aircraft resumes
 * an orbit on its own after a dropout. Leave the request latched and let the
 * gate block it. loiterStateInit() is for initialisation only.
 *
 * ``transitioned`` reports a change in the EFFECTIVE state, which is what the
 * caller must reset its attitude PIDs on. Watching the requested mode instead
 * would miss the airborne latch setting or clearing under a standing request
 * -- entering with integral wound against the pilot's setpoint, or handing the
 * stick back with the orbit's integral still applied -- and would also fire
 * spuriously while the pilot is hand-flying.
 */
static inline bool loiterUpdate(LoiterState* st,
                                bool requested,
                                bool rcFresh,
                                bool fbwActive,
                                bool attitudeUsable,
                                bool airborne,
                                float altitudeM,
                                bool altitudeValid)
{
    const bool wasRunning = st->running;
    const bool gatesOk =
        loiterMayEngage(requested, rcFresh, fbwActive, attitudeUsable, airborne);

    if (!requested) {
        /* Released: nothing flying, and the next request is a fresh edge. */
        st->running = false;
    } else if (!st->prevRequested) {
        /* Rising edge: engage only if every gate ALREADY passes. */
        st->running = gatesOk;
    } else {
        /* Standing request: keep flying only while the gates keep holding.
         * Once dropped it stays dropped until CH10 cycles. */
        st->running = st->running && gatesOk;
    }

    st->prevRequested = requested;
    st->transitioned = (st->running != wasRunning);

    if (st->transitioned) {
        if (st->running) {
            /* Capture the hold target at the moment the orbit starts. An
             * unusable barometer here means no altitude hold for this orbit --
             * pitch stays level, which is the un-held behaviour -- rather than
             * holding against a number that means nothing.
             *
             * Main.ino cannot actually reach that branch: it folds barometer
             * freshness into `airborne` (see loiterMayEngage), so an unusable
             * barometer fails the gate and there is no orbit to hold at all.
             * It stays because this header is caller-agnostic -- a caller
             * whose airborne latch does not depend on the barometer still
             * wants the un-held orbit rather than a hold on a stale number. */
            st->targetAltitudeM = altitudeM;
            st->targetAltitudeValid = altitudeValid;
        } else {
            st->targetAltitudeValid = false;
        }
    }

    return st->running;
}

/* Desired attitude for the orbit, in the FBW PIDs' own convention.
 *
 * Roll is the fixed bank. Pitch holds the altitude captured at engage, unless
 * anything about that is untrustworthy -- no captured target, an unusable
 * barometer now, no trustworthy airspeed, or airspeed below the floor -- in
 * which case it returns to level and the orbit simply descends as it would
 * without altitude hold. Every failure path degrades to the un-held orbit
 * rather than to a held one flying on bad numbers.
 *
 * Which of those the FIRMWARE can reach is narrower than the list. Main.ino
 * folds barometer freshness into the `airborne` gate, so both barometer paths
 * end the orbit outright instead of arriving here un-held; only the pitot
 * paths -- stale airspeed, or airspeed below the floor -- degrade to level
 * pitch in this build. Do not read the barometer branches as documentation of
 * what this aircraft does; they exist for callers that gate `airborne` on
 * something else.
 *
 * Both axes are clamped into the caller's FBW envelope, so editing the
 * constants above can never command past the limit the rest of the firmware
 * enforces.
 */
static inline void loiterDesiredAttitude(const LoiterState* st,
                                         float maxRollDeg,
                                         float maxPitchDeg,
                                         float altitudeM,
                                         bool altitudeValid,
                                         float airspeedMph,
                                         bool airspeedValid,
                                         float* desiredRollDeg,
                                         float* desiredPitchDeg)
{
    float roll = LOITER_BANK_ANGLE_DEG * LOITER_BANK_DIRECTION;
    float pitch = LOITER_PITCH_ANGLE_DEG;

    /* The airspeed floor outranks altitude: holding height matters less than
     * not flying the wing to a stall, and level pitch in a bank descends,
     * which is how the aircraft recovers the speed. */
    const bool speedOk = airspeedValid && (airspeedMph >= LOITER_MIN_AIRSPEED_MPH);
    const bool holdAltitude =
        (st != 0) && st->running && st->targetAltitudeValid && altitudeValid && speedOk;

    if (holdAltitude) {
        const float errorM = st->targetAltitudeM - altitudeM;
        pitch = LOITER_ALT_KP_DEG_PER_M * errorM;
        if (pitch > LOITER_ALT_PITCH_LIMIT_DEG) { pitch = LOITER_ALT_PITCH_LIMIT_DEG; }
        if (pitch < -LOITER_ALT_PITCH_LIMIT_DEG) { pitch = -LOITER_ALT_PITCH_LIMIT_DEG; }
    }

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

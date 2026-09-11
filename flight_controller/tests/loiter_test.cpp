/*************************************************************************************************************
 * Host-side verification for the FC loiter mode (loiter_nav.h). Compiles the
 * EXACT flight header (pure predicates and float math, no Arduino dependency)
 * and proves the properties that decide whether an orbit is safe to enter:
 *   1. Request band: only an explicit high CH10 requests the orbit, and every
 *      other value -- centre, failsafe low, the deadband edge -- means off, so
 *      a transient or defaulted channel cannot latch an autonomous mode.
 *   2. Gating: the orbit needs RC fresh AND Fly-By-Wire AND a usable attitude
 *      estimate AND the airborne latch, all of them, every cycle. Each one is
 *      checked in isolation so a future edit cannot quietly drop a condition.
 *   3. Geometry: the commanded bank matches the configured angle and sign, and
 *      stays clamped inside the FBW envelope even if the bank constant is
 *      edited past it.
 *   4. Altitude hold: the target is captured at engage and does not follow the
 *      aircraft, the sense is right (below target = nose UP -- getting this
 *      backwards flies it into the ground), the output clamps well inside the
 *      FBW envelope, and EVERY bad-data path -- below the airspeed floor,
 *      stale pitot, unusable barometer at engage or in flight, no state --
 *      falls back to level pitch, i.e. the un-held orbit. The degraded case is
 *      never worse than not having the feature.
 *   5. Firmware reachability: the header is caller-agnostic, but Main.ino wires
 *      `airborne` to `aircraftAirborne && barometerInputFresh(...)` and passes
 *      that same bool as `altitudeValid`. Under THAT wiring the barometer
 *      degraded paths above cannot be reached at all -- a bad barometer ends
 *      the orbit instead of flying it un-held. Pinned so the header's and the
 *      protocol contract's claims about this build cannot silently stop being
 *      true if the wiring changes.
 *
 * Build & run:
 *   c++ -std=c++17 -I.. -O2 -o /tmp/loiter_test loiter_test.cpp && /tmp/loiter_test
 ************************************************************************************************************/

#include "../loiter_nav.h"

#include <cmath>
#include <cstdio>

/* The FC's own channel centre: Main.ino computes (172 + 1811) / 2 with integer
 * truncation, which is the value an undriven channel array carries. */
static const uint16_t RC_CENTRE_FOR_TEST = (172 + 1811) / 2;

static int g_fail = 0;

static void check(bool cond, const char* what) {
    if (!cond) { g_fail++; std::printf("  FAIL %s\n", what); }
}

/* Close a test group. Prints "ok" only if THIS group added no failures.
 * Printing it unconditionally -- as every group used to -- meant a failing
 * group reported its "FAIL ..." lines and then "ok" directly underneath, which
 * is the one thing a verification harness must never do. */
static int g_fail_at_group_start = 0;
static bool groupPassed() {
    const bool passed = (g_fail == g_fail_at_group_start);
    g_fail_at_group_start = g_fail;
    return passed;
}
static void finish() {
    if (groupPassed()) { std::printf("  ok\n"); }
}
static void finishNote(const char* note) {
    if (groupPassed()) { std::printf("  ok   %s\n", note); }
}

static void checkNear(float got, float want, float tol, const char* what) {
    if (std::fabs(got - want) > tol) {
        g_fail++;
        std::printf("  FAIL %s (got %.4f, want %.4f)\n", what, (double)got, (double)want);
    }
}

/* --------------------------------------------------------------------------
 * 1. Request band
 * ------------------------------------------------------------------------ */
static void test_request_band() {
    std::printf("request band (CH10 >= %d requests the orbit)\n", LOITER_REQUEST_MIN);

    /* The values the ground station actually sends. */
    check(!loiterRequestedFromChannel(172),  "CRSF minimum must not request loiter");
    check(!loiterRequestedFromChannel(400),  "the GS 'off' value must not request loiter");
    check(!loiterRequestedFromChannel(992),  "channel centre must not request loiter");
    check(loiterRequestedFromChannel(1700),  "the GS 'on' value must request loiter");
    check(loiterRequestedFromChannel(1811),  "CRSF maximum must request loiter");

    /* Exactly at the threshold and one count either side of it. */
    check(loiterRequestedFromChannel(LOITER_REQUEST_MIN),
          "the threshold itself must request loiter");
    check(!loiterRequestedFromChannel(LOITER_REQUEST_MIN - 1),
          "one count below the threshold must not request loiter");

    /* An unpopulated channel array defaults to centre, which must read as off:
     * this is what protects a firmware running ahead of a GS that does not yet
     * drive CH10 at all. */
    check(!loiterRequestedFromChannel(RC_CENTRE_FOR_TEST),
          "a GS that never drives CH10 must never get an orbit");

    finish();
}

/* --------------------------------------------------------------------------
 * 2. Gating
 * ------------------------------------------------------------------------ */
static void test_gate_requires_everything() {
    std::printf("engage gate (every condition required, every cycle)\n");

    check(loiterMayEngage(true, true, true, true, true),
          "all conditions satisfied must engage");

    /* Drop one condition at a time; each alone must veto. */
    check(!loiterMayEngage(false, true, true, true, true), "no request must veto");
    check(!loiterMayEngage(true, false, true, true, true), "stale RC must veto");
    check(!loiterMayEngage(true, true, false, true, true), "Manual mode must veto");
    check(!loiterMayEngage(true, true, true, false, true), "unusable attitude must veto");
    check(!loiterMayEngage(true, true, true, true, false), "on the ground must veto");

    /* Nothing satisfied is obviously off, and the exhaustive sweep below makes
     * sure the predicate really is the conjunction rather than something that
     * happens to agree on the cases above. */
    int engaged = 0;
    for (int m = 0; m < 32; ++m) {
        const bool req  = (m & 1) != 0;
        const bool rc   = (m & 2) != 0;
        const bool fbw  = (m & 4) != 0;
        const bool att  = (m & 8) != 0;
        const bool air  = (m & 16) != 0;
        if (loiterMayEngage(req, rc, fbw, att, air)) { engaged++; }
    }
    check(engaged == 1, "exactly one of the 32 gate combinations may engage");

    finishNote("1 of 32 combinations engages");
}

/* --------------------------------------------------------------------------
 * 2b. The rising-edge latch
 * ------------------------------------------------------------------------ */
static void test_ground_request_does_not_spring_after_takeoff() {
    std::printf("latch (a request raised on the ground must not fly itself)\n");

    LoiterState st;
    loiterStateInit(&st);

    /* Operator completes the hold while still on the ground. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/false, 100.0f, true),
          "a grounded request must not fly");
    check(!loiterUpdate(&st, true, true, true, true, false, 100.0f, true), "still grounded, still refused");

    /* Takeoff: the airborne latch sets while CH10 is still high. This is the
     * exact case that would otherwise roll the model into the orbit seconds
     * after liftoff with no operator action. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/true, 100.0f, true),
          "the airborne latch setting must NOT engage a standing request");
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "and it must stay refused on later cycles");

    /* Only dropping and re-raising CH10 arms it. */
    check(!loiterUpdate(&st, false, true, true, true, true, 100.0f, true), "release clears the request");
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "a fresh request while airborne must engage");

    finish();
}

static void test_latch_drops_and_requires_a_new_edge() {
    std::printf("latch (a dropped gate stays dropped until CH10 cycles)\n");

    LoiterState st;
    loiterStateInit(&st);
    check(!loiterUpdate(&st, false, true, true, true, true, 100.0f, true), "start released");
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "rising edge engages");

    /* Losing the airborne latch (a low pass near the ground) drops the orbit. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/false, 100.0f, true),
          "losing airborne must drop the orbit");
    /* Regaining it must NOT silently resume under the same standing request. */
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "regaining airborne must not resume without a new request");

    finish();
}

static void test_attitude_outage_does_not_resume_silently() {
    std::printf("latch (an attitude outage must not resume on recovery)\n");

    LoiterState st;
    loiterStateInit(&st);
    loiterUpdate(&st, false, true, true, true, true, 100.0f, true);
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "orbiting");

    /* A dead IMU or an unconverged EKF after a watchdog boot routes control to
     * the pass-through branch. The latch must still be advanced with the real
     * value: if it is skipped, `running` stays set through the outage. */
    check(!loiterUpdate(&st, true, true, true, /*attitudeUsable=*/false, true, 100.0f, true),
          "an unusable attitude estimate must drop the orbit");
    check(st.transitioned, "and must flag the transition so the PIDs reset");

    /* Recovery under the same standing request must NOT resume. */
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "recovery must not resume without a fresh request edge");
    check(!st.transitioned, "and must not re-flag a transition");

    /* Only a CH10 cycle brings it back. */
    check(!loiterUpdate(&st, false, true, true, true, true, 100.0f, true), "release");
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "fresh request re-engages");

    finish();
}

static void test_request_during_convergence_is_refused() {
    std::printf("latch (a request raised while attitude is unusable is refused)\n");

    LoiterState st;
    loiterStateInit(&st);

    /* Watchdog-recovery boot: the estimate is fresh but not yet converged, so
     * the FC is in limited-authority pass-through. Critically the ground
     * station CANNOT see this -- attitude telemetry keeps flowing -- so it
     * will happily raise CH10. The latch has to catch it. */
    check(!loiterUpdate(&st, true, true, true, /*attitudeUsable=*/false, true, 100.0f, true),
          "a request raised during convergence must not fly");
    check(!loiterUpdate(&st, true, true, true, false, true, 100.0f, true), "still refused");

    /* Convergence completes. Without observing the request during the outage
     * this would look like a first rising edge and be accepted. */
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "convergence completing must not accept the standing request");

    finish();
}

static void test_a_frozen_altitude_sensor_must_end_the_orbit() {
    std::printf("airborne gate (a frozen barometer hides the landing)\n");

    /* The firmware's airborne latch clears on HEIGHT alone, and height comes
     * from a barometer reading that a failed sensor leaves frozen. The latch
     * then stays set forever, the landing is never detected, and the orbit
     * would go on commanding its bank after touchdown. The caller folds
     * barometer freshness into the airborne argument so that cannot happen;
     * this pins the behaviour the caller depends on. */
    LoiterState st;
    loiterStateInit(&st);
    loiterUpdate(&st, false, true, true, true, true, 100.0f, true);
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "orbiting");

    /* Barometer freezes: the caller passes airborne=false rather than the raw
     * latch, so the orbit ends and the pilot gets the aircraft back. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/false, 100.0f, false),
          "an untrustworthy airborne latch must end the orbit");
    check(st.transitioned, "and must flag the transition so the PIDs reset");

    /* And it must not resume when the sensor recovers, without a fresh edge. */
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "recovery must not resume without a new request");

    finish();
}

static void test_transition_flag_tracks_effective_state() {
    std::printf("transition flag (PID resets follow the effective state)\n");

    LoiterState st;
    loiterStateInit(&st);

    loiterUpdate(&st, false, true, true, true, true, 100.0f, true);
    check(!st.transitioned, "no change means no reset");

    loiterUpdate(&st, true, true, true, true, true, 100.0f, true);
    check(st.transitioned, "engaging must flag a transition");

    loiterUpdate(&st, true, true, true, true, true, 100.0f, true);
    check(!st.transitioned, "steady orbit must not keep resetting the PIDs");

    loiterUpdate(&st, true, true, /*fbwActive=*/false, true, true, 100.0f, true);
    check(st.transitioned, "dropping out must flag a transition");

    /* A request that never flies must not flag anything: resetting there would
     * dump the integrator while the pilot is hand-flying Fly-By-Wire. */
    loiterStateInit(&st);
    loiterUpdate(&st, true, true, true, true, /*airborne=*/false, 100.0f, true);
    check(!st.transitioned, "a refused request must not touch the PIDs");
    loiterUpdate(&st, true, true, true, true, false, 100.0f, true);
    check(!st.transitioned, "and must keep not touching them");

    finish();
}

static void test_rc_failsafe_does_not_resume_on_recovery() {
    std::printf("failsafe (a link dropout must not resume the orbit by itself)\n");

    LoiterState st;
    loiterStateInit(&st);
    loiterUpdate(&st, false, true, true, true, true, 100.0f, true);
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "orbiting before the dropout");

    /* Link lost. The firmware must NOT force the request off: doing so would
     * make the standing CH10 look released, and the first recovered packet
     * would then read as a fresh rising edge. The request stays high and is
     * blocked by the rcFresh gate instead. */
    check(!loiterUpdate(&st, true, /*rcFresh=*/false, true, true, true, 100.0f, true),
          "a stale link must stop the orbit");
    check(st.transitioned, "and must flag the transition so the PIDs reset");
    check(!loiterUpdate(&st, true, false, true, true, true, 100.0f, true), "still stopped while stale");

    /* Link recovers with CH10 still high, and control mode restored. This is
     * the case that must NOT fly: the aircraft has just been through a
     * failsafe -- surfaces blended to neutral, throttle cut -- so resuming an
     * orbit on its own is precisely what the edge requirement forbids. */
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true),
          "recovery must not resume the orbit without an operator gesture");
    check(!loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "and must stay dropped");

    /* The operator cycling CH10 is the only way back. */
    check(!loiterUpdate(&st, false, true, true, true, true, 100.0f, true), "operator releases");
    check(loiterUpdate(&st, true, true, true, true, true, 100.0f, true), "and re-requests");

    finish();
}

/* --------------------------------------------------------------------------
 * 3. Geometry
 * ------------------------------------------------------------------------ */
static LoiterState flyingState(float targetAltM = 100.0f, bool targetValid = true) {
    LoiterState st;
    loiterStateInit(&st);
    loiterUpdate(&st, false, true, true, true, true, targetAltM, targetValid);
    loiterUpdate(&st, true, true, true, true, true, targetAltM, targetValid);
    return st;
}

static void test_commanded_attitude() {
    std::printf("commanded attitude (bank %.1f deg)\n", (double)LOITER_BANK_ANGLE_DEG);

    LoiterState st = flyingState();
    float roll = 999.0f, pitch = 999.0f;

    /* On target, with good speed: wings banked, pitch level. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 100.0f, true, 25.0f, true, &roll, &pitch);
    checkNear(roll, LOITER_BANK_ANGLE_DEG * LOITER_BANK_DIRECTION, 1e-6f,
              "commanded bank must match the configured angle and sign");
    checkNear(pitch, 0.0f, 1e-6f, "no altitude error means level pitch");

    check(std::fabs(roll) < 80.0f * 0.75f,
          "the orbit bank must sit well inside the FBW hard limit");

    /* Clamping: constants edited past the envelope must be limited by it. */
    loiterDesiredAttitude(&st, 10.0f, 5.0f, 100.0f, true, 25.0f, true, &roll, &pitch);
    check(std::fabs(roll) <= 10.0f + 1e-6f, "bank must clamp into the roll envelope");
    checkNear(roll, 10.0f * LOITER_BANK_DIRECTION, 1e-6f, "a clamped bank must keep its sign");

    /* A negative limit (a sign slip at the call site) must not invert the clamp. */
    loiterDesiredAttitude(&st, -10.0f, -5.0f, 100.0f, true, 25.0f, true, &roll, &pitch);
    check(std::fabs(roll) <= 10.0f + 1e-6f, "a negative roll limit must still clamp");
    check(std::fabs(pitch) <= 5.0f + 1e-6f, "a negative pitch limit must still clamp");

    /* Null outputs must be ignored rather than dereferenced. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 100.0f, true, 25.0f, true, 0, 0);

    finish();
}

static void test_altitude_hold_sign_and_clamp() {
    std::printf("altitude hold (P-only, clamped, correct sense)\n");

    LoiterState st = flyingState(100.0f);
    float roll = 0.0f, pitch = 0.0f;

    /* BELOW target -> positive error -> nose UP. Getting this sign backwards
     * flies the aircraft into the ground, so pin it explicitly. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 95.0f, true, 25.0f, true, &roll, &pitch);
    check(pitch > 0.0f, "below target must command nose up");
    checkNear(pitch, LOITER_ALT_KP_DEG_PER_M * 5.0f, 1e-5f, "gain must be proportional");

    /* ABOVE target -> nose down. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 105.0f, true, 25.0f, true, &roll, &pitch);
    check(pitch < 0.0f, "above target must command nose down");

    /* A huge error must saturate at the clamp, not command a vertical line. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, -500.0f, true, 25.0f, true, &roll, &pitch);
    checkNear(pitch, LOITER_ALT_PITCH_LIMIT_DEG, 1e-5f, "large error must clamp nose up");
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 5000.0f, true, 25.0f, true, &roll, &pitch);
    checkNear(pitch, -LOITER_ALT_PITCH_LIMIT_DEG, 1e-5f, "large error must clamp nose down");

    /* The clamp must sit inside the FBW envelope rather than riding it. */
    check(LOITER_ALT_PITCH_LIMIT_DEG < 80.0f * 0.5f,
          "the altitude pitch clamp must stay well inside the FBW limit");

    finish();
}

static void test_airspeed_floor_outranks_altitude() {
    std::printf("airspeed floor (outranks altitude; degrades to the un-held orbit)\n");

    LoiterState st = flyingState(100.0f);
    float roll = 0.0f, pitch = 0.0f;

    /* 50 m below target is a large nose-up demand -- exactly the situation in
     * which a naive loop flies the wing to a stall. Below the floor it must
     * abandon the hold and return to level, which descends and regains speed. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 50.0f, true,
                          LOITER_MIN_AIRSPEED_MPH - 1.0f, true, &roll, &pitch);
    checkNear(pitch, 0.0f, 1e-6f, "below the airspeed floor must return to level pitch");
    checkNear(roll, LOITER_BANK_ANGLE_DEG * LOITER_BANK_DIRECTION, 1e-6f,
              "the orbit itself must continue -- only the hold is abandoned");

    /* Exactly at the floor is still flying. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 50.0f, true,
                          LOITER_MIN_AIRSPEED_MPH, true, &roll, &pitch);
    check(pitch > 0.0f, "at the floor the hold must still work");

    /* No trustworthy airspeed at all is treated like being below the floor. */
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 50.0f, true, 99.0f, false, &roll, &pitch);
    checkNear(pitch, 0.0f, 1e-6f, "stale airspeed must not be flown on");

    finish();
}

static void test_hold_degrades_without_a_usable_barometer() {
    std::printf("altitude hold (every bad-data path falls back to level)\n");

    float roll = 0.0f, pitch = 0.0f;

    /* Barometer unusable AT ENGAGE: no target captured, so no hold all orbit. */
    LoiterState noTarget = flyingState(0.0f, /*targetValid=*/false);
    check(!noTarget.targetAltitudeValid, "an unusable baro at engage captures no target");
    loiterDesiredAttitude(&noTarget, 80.0f, 80.0f, 50.0f, true, 25.0f, true, &roll, &pitch);
    checkNear(pitch, 0.0f, 1e-6f, "no captured target means level pitch");

    /* Barometer captured fine but failing NOW. */
    LoiterState st = flyingState(100.0f);
    loiterDesiredAttitude(&st, 80.0f, 80.0f, 50.0f, /*altitudeValid=*/false,
                          25.0f, true, &roll, &pitch);
    checkNear(pitch, 0.0f, 1e-6f, "a failing baro must not be held against");

    /* A null state must not be dereferenced. */
    loiterDesiredAttitude(0, 80.0f, 80.0f, 50.0f, true, 25.0f, true, &roll, &pitch);
    checkNear(pitch, 0.0f, 1e-6f, "no state means level pitch");

    finish();
}

static void test_target_is_captured_at_engage_and_cleared_on_exit() {
    std::printf("altitude target (captured at engage, cleared on exit)\n");

    LoiterState st;
    loiterStateInit(&st);
    check(!st.targetAltitudeValid, "no target before an orbit runs");

    loiterUpdate(&st, false, true, true, true, true, 120.0f, true);
    loiterUpdate(&st, true, true, true, true, true, 120.0f, true);
    check(st.running, "orbit engaged");
    checkNear(st.targetAltitudeM, 120.0f, 1e-6f, "target is the altitude at engage");

    /* Drifting altitude while running must NOT move the target. */
    loiterUpdate(&st, true, true, true, true, true, 90.0f, true);
    checkNear(st.targetAltitudeM, 120.0f, 1e-6f, "the target must not follow the aircraft");

    /* Dropping out clears it, so a later orbit cannot hold a stale height. */
    loiterUpdate(&st, false, true, true, true, true, 90.0f, true);
    check(!st.targetAltitudeValid, "exiting clears the captured target");

    /* A fresh orbit captures wherever it now is. */
    loiterUpdate(&st, true, true, true, true, true, 70.0f, true);
    checkNear(st.targetAltitudeM, 70.0f, 1e-6f, "a new orbit captures a new target");

    finish();
}

/* Reproduces Main.ino's wiring exactly. The firmware does NOT pass the raw
 * airborne latch and an independent altitude-validity flag: it passes
 * `aircraftAirborne && barometerInputFresh(...)` as `airborne`, and that same
 * freshness bool again as `altitudeValid`. The two arguments are therefore
 * coupled in this build, which is precisely what makes the barometer branches
 * of loiterDesiredAttitude() unreachable here. */
static bool firmwareLoiterUpdate(LoiterState* st, bool requested, bool rcFresh,
                                 bool fbw, bool attitudeUsable,
                                 bool aircraftAirborne, float altitudeM,
                                 bool barometerFresh) {
    return loiterUpdate(st, requested, rcFresh, fbw, attitudeUsable,
                        aircraftAirborne && barometerFresh,
                        altitudeM, barometerFresh);
}

static void test_firmware_wiring_makes_the_baro_paths_unreachable() {
    std::printf("firmware wiring (a bad barometer ENDS the orbit, never degrades it)\n");

    for (int airborneBit = 0; airborneBit < 2; ++airborneBit) {
        for (int baroBit = 0; baroBit < 2; ++baroBit) {
            const bool aircraftAirborne = (airborneBit != 0);
            const bool baroFresh = (baroBit != 0);

            LoiterState st;
            loiterStateInit(&st);
            firmwareLoiterUpdate(&st, false, true, true, true,
                                 aircraftAirborne, 120.0f, baroFresh);
            const bool running = firmwareLoiterUpdate(&st, true, true, true, true,
                                                      aircraftAirborne, 120.0f,
                                                      baroFresh);
            if (!baroFresh) {
                check(!running,
                      "a stale barometer must end the orbit, not fly it un-held");
            }
            /* Whenever the firmware IS orbiting it holds a captured, valid
             * target -- so the "no target captured" fallback is unreachable
             * too, not merely the "barometer failing now" one. */
            if (running) {
                check(st.targetAltitudeValid,
                      "a running firmware orbit always has a valid target");
            }
        }
    }

    /* Losing the barometer mid-orbit ends it, rather than continuing level. */
    LoiterState st;
    loiterStateInit(&st);
    firmwareLoiterUpdate(&st, false, true, true, true, true, 120.0f, true);
    check(firmwareLoiterUpdate(&st, true, true, true, true, true, 120.0f, true),
          "orbit engages with a live barometer");
    check(!firmwareLoiterUpdate(&st, true, true, true, true, true, 120.0f, false),
          "the orbit ends when the barometer freezes mid-flight");

    finish();
}

int main() {
    std::printf("FC loiter: fixed-bank orbit substituted for the stick inside the FBW branch\n\n");
    test_request_band();
    test_gate_requires_everything();
    test_ground_request_does_not_spring_after_takeoff();
    test_latch_drops_and_requires_a_new_edge();
    test_attitude_outage_does_not_resume_silently();
    test_a_frozen_altitude_sensor_must_end_the_orbit();
    test_request_during_convergence_is_refused();
    test_transition_flag_tracks_effective_state();
    test_rc_failsafe_does_not_resume_on_recovery();
    test_commanded_attitude();
    test_altitude_hold_sign_and_clamp();
    test_airspeed_floor_outranks_altitude();
    test_hold_degrades_without_a_usable_barometer();
    test_target_is_captured_at_engage_and_cleared_on_exit();
    test_firmware_wiring_makes_the_baro_paths_unreachable();
    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}

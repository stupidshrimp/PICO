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
 *      edited past it. Pitch is level.
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

    std::printf("  ok\n");
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

    std::printf("  ok   1 of 32 combinations engages\n");
}

/* --------------------------------------------------------------------------
 * 2b. The rising-edge latch
 * ------------------------------------------------------------------------ */
static void test_ground_request_does_not_spring_after_takeoff() {
    std::printf("latch (a request raised on the ground must not fly itself)\n");

    LoiterState st;
    loiterStateInit(&st);

    /* Operator completes the hold while still on the ground. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/false),
          "a grounded request must not fly");
    check(!loiterUpdate(&st, true, true, true, true, false), "still grounded, still refused");

    /* Takeoff: the airborne latch sets while CH10 is still high. This is the
     * exact case that would otherwise roll the model into a 20 degree orbit
     * seconds after liftoff with no operator action. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/true),
          "the airborne latch setting must NOT engage a standing request");
    check(!loiterUpdate(&st, true, true, true, true, true),
          "and it must stay refused on later cycles");

    /* Only dropping and re-raising CH10 arms it. */
    check(!loiterUpdate(&st, false, true, true, true, true), "release clears the request");
    check(loiterUpdate(&st, true, true, true, true, true),
          "a fresh request while airborne must engage");

    std::printf("  ok\n");
}

static void test_latch_drops_and_requires_a_new_edge() {
    std::printf("latch (a dropped gate stays dropped until CH10 cycles)\n");

    LoiterState st;
    loiterStateInit(&st);
    check(!loiterUpdate(&st, false, true, true, true, true), "start released");
    check(loiterUpdate(&st, true, true, true, true, true), "rising edge engages");

    /* Losing the airborne latch (a low pass near the ground) drops the orbit. */
    check(!loiterUpdate(&st, true, true, true, true, /*airborne=*/false),
          "losing airborne must drop the orbit");
    /* Regaining it must NOT silently resume under the same standing request. */
    check(!loiterUpdate(&st, true, true, true, true, true),
          "regaining airborne must not resume without a new request");

    std::printf("  ok\n");
}

static void test_attitude_outage_does_not_resume_silently() {
    std::printf("latch (an attitude outage must not resume on recovery)\n");

    LoiterState st;
    loiterStateInit(&st);
    loiterUpdate(&st, false, true, true, true, true);
    check(loiterUpdate(&st, true, true, true, true, true), "orbiting");

    /* A dead IMU or an unconverged EKF after a watchdog boot routes control to
     * the pass-through branch. The latch must still be advanced with the real
     * value: if it is skipped, `running` stays set through the outage. */
    check(!loiterUpdate(&st, true, true, true, /*attitudeUsable=*/false, true),
          "an unusable attitude estimate must drop the orbit");
    check(st.transitioned, "and must flag the transition so the PIDs reset");

    /* Recovery under the same standing request must NOT resume. */
    check(!loiterUpdate(&st, true, true, true, true, true),
          "recovery must not resume without a fresh request edge");
    check(!st.transitioned, "and must not re-flag a transition");

    /* Only a CH10 cycle brings it back. */
    check(!loiterUpdate(&st, false, true, true, true, true), "release");
    check(loiterUpdate(&st, true, true, true, true, true), "fresh request re-engages");

    std::printf("  ok\n");
}

static void test_request_during_convergence_is_refused() {
    std::printf("latch (a request raised while attitude is unusable is refused)\n");

    LoiterState st;
    loiterStateInit(&st);

    /* Watchdog-recovery boot: the estimate is fresh but not yet converged, so
     * the FC is in limited-authority pass-through. Critically the ground
     * station CANNOT see this -- attitude telemetry keeps flowing -- so it
     * will happily raise CH10. The latch has to catch it. */
    check(!loiterUpdate(&st, true, true, true, /*attitudeUsable=*/false, true),
          "a request raised during convergence must not fly");
    check(!loiterUpdate(&st, true, true, true, false, true), "still refused");

    /* Convergence completes. Without observing the request during the outage
     * this would look like a first rising edge and be accepted. */
    check(!loiterUpdate(&st, true, true, true, true, true),
          "convergence completing must not accept the standing request");

    std::printf("  ok\n");
}

static void test_transition_flag_tracks_effective_state() {
    std::printf("transition flag (PID resets follow the effective state)\n");

    LoiterState st;
    loiterStateInit(&st);

    loiterUpdate(&st, false, true, true, true, true);
    check(!st.transitioned, "no change means no reset");

    loiterUpdate(&st, true, true, true, true, true);
    check(st.transitioned, "engaging must flag a transition");

    loiterUpdate(&st, true, true, true, true, true);
    check(!st.transitioned, "steady orbit must not keep resetting the PIDs");

    loiterUpdate(&st, true, true, /*fbwActive=*/false, true, true);
    check(st.transitioned, "dropping out must flag a transition");

    /* A request that never flies must not flag anything: resetting there would
     * dump the integrator while the pilot is hand-flying Fly-By-Wire. */
    loiterStateInit(&st);
    loiterUpdate(&st, true, true, true, true, /*airborne=*/false);
    check(!st.transitioned, "a refused request must not touch the PIDs");
    loiterUpdate(&st, true, true, true, true, false);
    check(!st.transitioned, "and must keep not touching them");

    std::printf("  ok\n");
}

static void test_rc_failsafe_does_not_resume_on_recovery() {
    std::printf("failsafe (a link dropout must not resume the orbit by itself)\n");

    LoiterState st;
    loiterStateInit(&st);
    loiterUpdate(&st, false, true, true, true, true);
    check(loiterUpdate(&st, true, true, true, true, true), "orbiting before the dropout");

    /* Link lost. The firmware must NOT force the request off: doing so would
     * make the standing CH10 look released, and the first recovered packet
     * would then read as a fresh rising edge. The request stays high and is
     * blocked by the rcFresh gate instead. */
    check(!loiterUpdate(&st, true, /*rcFresh=*/false, true, true, true),
          "a stale link must stop the orbit");
    check(st.transitioned, "and must flag the transition so the PIDs reset");
    check(!loiterUpdate(&st, true, false, true, true, true), "still stopped while stale");

    /* Link recovers with CH10 still high, and control mode restored. This is
     * the case that must NOT fly: the aircraft has just been through a
     * failsafe -- surfaces blended to neutral, throttle cut -- so resuming an
     * orbit on its own is precisely what the edge requirement forbids. */
    check(!loiterUpdate(&st, true, true, true, true, true),
          "recovery must not resume the orbit without an operator gesture");
    check(!loiterUpdate(&st, true, true, true, true, true), "and must stay dropped");

    /* The operator cycling CH10 is the only way back. */
    check(!loiterUpdate(&st, false, true, true, true, true), "operator releases");
    check(loiterUpdate(&st, true, true, true, true, true), "and re-requests");

    std::printf("  ok\n");
}

/* --------------------------------------------------------------------------
 * 3. Geometry
 * ------------------------------------------------------------------------ */
static void test_commanded_attitude() {
    std::printf("commanded attitude (bank %.1f deg, level pitch)\n",
                (double)LOITER_BANK_ANGLE_DEG);

    float roll = 999.0f, pitch = 999.0f;
    loiterDesiredAttitude(80.0f, 80.0f, &roll, &pitch);

    checkNear(roll, LOITER_BANK_ANGLE_DEG * LOITER_BANK_DIRECTION, 1e-6f,
              "commanded bank must match the configured angle and sign");
    checkNear(pitch, LOITER_PITCH_ANGLE_DEG, 1e-6f, "commanded pitch must be level");

    /* The bank must land well inside the FBW hard limit, so that limit stays a
     * redundant backstop rather than something the orbit rides against. */
    check(std::fabs(roll) < 80.0f * 0.75f,
          "the orbit bank must sit well inside the FBW hard limit");

    /* Clamping: a bank constant edited past the envelope must be limited by it
     * rather than commanding through it. */
    roll = pitch = 999.0f;
    loiterDesiredAttitude(10.0f, 5.0f, &roll, &pitch);
    check(std::fabs(roll) <= 10.0f + 1e-6f, "bank must clamp into the roll envelope");
    check(std::fabs(pitch) <= 5.0f + 1e-6f, "pitch must clamp into the pitch envelope");
    checkNear(roll, 10.0f * LOITER_BANK_DIRECTION, 1e-6f,
              "a clamped bank must keep its sign");

    /* A negative limit (a sign slip at the call site) must not invert the
     * clamp into an unbounded command. */
    roll = pitch = 999.0f;
    loiterDesiredAttitude(-10.0f, -5.0f, &roll, &pitch);
    check(std::fabs(roll) <= 10.0f + 1e-6f, "a negative roll limit must still clamp");
    check(std::fabs(pitch) <= 5.0f + 1e-6f, "a negative pitch limit must still clamp");

    /* Null outputs must be ignored rather than dereferenced. */
    loiterDesiredAttitude(80.0f, 80.0f, 0, 0);

    std::printf("  ok\n");
}

int main() {
    std::printf("FC loiter: fixed-bank orbit substituted for the stick inside the FBW branch\n\n");
    test_request_band();
    test_gate_requires_everything();
    test_ground_request_does_not_spring_after_takeoff();
    test_latch_drops_and_requires_a_new_edge();
    test_attitude_outage_does_not_resume_silently();
    test_request_during_convergence_is_refused();
    test_transition_flag_tracks_effective_state();
    test_rc_failsafe_does_not_resume_on_recovery();
    test_commanded_attitude();
    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}

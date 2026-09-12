#ifndef FEATHER_CONTROL_MODE_H
#define FEATHER_CONTROL_MODE_H

// Keep ControlMode in a header so Arduino's generated function prototypes see
// the type before any sketch functions that use it (for example setControlMode).
enum ControlMode {
  CONTROL_MODE_MANUAL = 0,
  CONTROL_MODE_FLY_BY_WIRE
};

enum ThrottleMode {
  THROTTLE_MODE_MANUAL = 0,
  THROTTLE_MODE_AUTO
};

// Autonomous navigation mode layered on top of Fly-By-Wire. NAV_MODE_LOITER
// substitutes a fixed bank/level pitch for the pilot's stick inside the FBW
// branch; see loiter_nav.h. Manual control is unaffected -- nav modes only
// have meaning while the FBW PIDs are the ones flying.
enum NavMode {
  NAV_MODE_OFF = 0,
  NAV_MODE_LOITER
};

#endif // FEATHER_CONTROL_MODE_H

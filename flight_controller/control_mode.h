#ifndef FEATHER_CONTROL_MODE_H
#define FEATHER_CONTROL_MODE_H

// Keep ControlMode in a header so Arduino's generated function prototypes see
// the type before any sketch functions that use it (for example setControlMode).
enum ControlMode {
  CONTROL_MODE_MANUAL = 0,
  CONTROL_MODE_FLY_BY_WIRE
};

#endif // FEATHER_CONTROL_MODE_H

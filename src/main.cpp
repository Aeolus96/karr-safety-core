// ============================================================================
// K.A.R.R. (Keystone Autonomous Resource Rover) — safety & relay controller
// Firmware project: ESP32-RELAY-FW (e-stop + lighting relay baseline)
// ============================================================================
// Arduino entry points only. All wiring, boot sequencing, the control task
// and the cooperative micro-ROS loop live in the karr modules — see
// src/app.cpp for orchestration and README.md for the module map.
// ============================================================================

#include <Arduino.h>

#include "karr/app.h"

void setup()
{
    karr::app::setup();
}

void loop()
{
    karr::app::loop();
}

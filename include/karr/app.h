#pragma once

// ============================================================================
// Application orchestration — wiring and execution contexts.
// Owns the concrete subsystem instances (construction order preserved from
// the original single-file firmware), the boot sequencing, the control-task
// startup, and the cooperative ROS loop. No policy logic lives here.
// ============================================================================

namespace karr
{
namespace app
{

// Arduino entry points (called from src/main.cpp).
void setup();
void loop();

} // namespace app
} // namespace karr

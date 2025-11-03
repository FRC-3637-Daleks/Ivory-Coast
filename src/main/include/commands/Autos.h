#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveBase.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(DriveBase* subsystem);
}  // namespace autos

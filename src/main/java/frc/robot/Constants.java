// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final boolean isGuido = false;

  public enum RobotType {
    SIMBOT,
    GYARADOS,
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final DriverControl driverControl = DriverControl.ALIGN;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum DriverControl {
    /** Auto-align to closest goal */
    ALIGN,

    /** Free control */
    FREE
  }

  public static final double loopPeriodSecs = Robot.defaultPeriodSecs;

  public static final double phoenixUpdateFreqHz = 50.0;
  public static final String alternateCanBus = isGuido ? "rio" : "SUSBus";

  public static final double controllerDeadband = 0.15;
  public static final double triggerPressedThreshold = 0.1;
}

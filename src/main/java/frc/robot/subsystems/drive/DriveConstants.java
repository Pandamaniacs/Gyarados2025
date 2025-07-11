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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // TODO: max speed, wheel radius, gyro trimming
  public static final double maxSpeedMetersPerSec = 4.4;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22.5);
  public static final double wheelBase = trackWidth;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module.
  // When calibrating, hold the metal bar to each wheel,
  // using the flat side (without gear) to the right of the robot
  public static final double frontLeftZeroOffset = -21.40;
  public static final double frontRightZeroOffset = 11.27;
  public static final double backLeftZeroOffset = 119.85;
  public static final double backRightZeroOffset = 128.68;

  public static final Rotation2d frontLeftZeroRotation =
      Rotation2d.fromDegrees(frontLeftZeroOffset);
  public static final Rotation2d frontRightZeroRotation =
      Rotation2d.fromDegrees(frontRightZeroOffset);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromDegrees(backLeftZeroOffset);
  public static final Rotation2d backRightZeroRotation =
      Rotation2d.fromDegrees(backRightZeroOffset);

  public static final double gyroTrimScalar = 0.0;

  // Device CAN IDs
  public static final int pigeonCanId = 1;
  public static final double mountPoseYawDeg = 0;
  public static final double mountPosePitchDeg = 0;
  public static final double mountPoseRollDeg = 0;

  public static final int frontLeftDriveCanId = 1;
  public static final int frontRightDriveCanId = 2;
  public static final int backLeftDriveCanId = 3;
  public static final int backRightDriveCanId = 4;

  public static final int frontLeftTurnCanId = 5;
  public static final int frontRightTurnCanId = 6;
  public static final int backLeftTurnCanId = 7;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftAbsoluteInputPort = 0;
  public static final int frontRightAbsoluteInputPort = 1;
  public static final int backLeftAbsoluteInputPort = 2;
  public static final int backRightAbsoluteInputPort = 3;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.4797244566677281);
  // MAXSwerve with 12 pinion teeth and 22 spur teeth
  public static final double driveMotorReduction = (45.0 * 22.0) / 13.0 * 15.0;
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.13014;
  public static final double driveKv = 0.09750;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  // Pulled from SDS Documentation
  public static final double turnMotorReduction = 12.8;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      2 * Math.PI / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
}

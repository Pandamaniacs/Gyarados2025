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

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOGyarados;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.util.CommandCustomXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;

  // Controller
  public final CommandCustomXboxController driverController = new CommandCustomXboxController(0);

  // Dashboard inputs
  private final AutoChooser autoChooser = new AutoChooser();
  //   private final Autos autos;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOGyarados(0),
                new ModuleIOGyarados(1),
                new ModuleIOGyarados(2),
                new ModuleIOGyarados(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      case REPLAY:
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addCmd(
        "Drive Wheel Radius Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureRotationModes();
  }

  private void configureRotationModes() {
    switch (Constants.driverControl) {
      case ALIGN:
        drive.setDefaultCommand(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftYSquared(),
                    () -> -driverController.getLeftXSquared(),
                    () -> BobotState.getCurrentAlignmentTracker().getRotationTarget())
                .unless(DriverStation::isAutonomous));
        break;
      case FREE:
      default:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftYSquared(),
                () -> -driverController.getLeftXSquared(),
                () -> driverController.getRightXSquared()));
        break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}

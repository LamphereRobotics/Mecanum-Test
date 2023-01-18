// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DriveSubsystem drive;

  private final Joystick leftDriveJoystick;
  private final Joystick rightDriveJoystick;

  public RobotContainer() {
    drive = new DriveSubsystem();

    leftDriveJoystick = new Joystick(0);
    rightDriveJoystick = new Joystick(1);

    configureJoysticks();
    configureBindings();
    configureDefaultCommands();
  }

  private void configureJoysticks() {

  }

  private void configureBindings() {

  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.drive(
                leftDriveJoystick.getY(),
                leftDriveJoystick.getX(),
                rightDriveJoystick.getX(),
                !leftDriveJoystick.getRawButton(0)),
            drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

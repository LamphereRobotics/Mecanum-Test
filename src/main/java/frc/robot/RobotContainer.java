// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.Joysticks.Axes.*;
import static frc.robot.Constants.Joysticks.Buttons.*;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    toggleFieldRelativeButton
        .onTrue(drive.toggleFieldRelativeCommand());
    minSpeedButton
        .onTrue(drive.setSpeedLimitCommand(0.5))
        .onFalse(drive.setSpeedLimitCommand(5));
    lowSpeedButton
        .onTrue(drive.setSpeedLimitCommand(2.5))
        .onFalse(drive.setSpeedLimitCommand(5));
    highSpeedButton
        .onTrue(drive.setSpeedLimitCommand(10))
        .onFalse(drive.setSpeedLimitCommand(5));
    gyroResetButton
    .onTrue(new InstantCommand(drive::resetGyro));
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.driveCommand(driveForwardAxis, driveLeftAxis, rotateAxis));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

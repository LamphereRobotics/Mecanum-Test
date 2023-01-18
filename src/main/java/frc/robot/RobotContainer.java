// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        .onTrue(drive.setFieldRelativeCommand(true))
        .onFalse(drive.setFieldRelativeCommand(false));
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.driveCommand(driveForwardAxis, driveLeftAxis, rotateAxis));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

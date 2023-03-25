// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.ConeAuton;
import frc.robot.Commands.MidAuton;
import frc.robot.Commands.RightAuton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.Joysticks.Axes.*;
import static frc.robot.Constants.Joysticks.Buttons.*;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();
  private final Intake intake = new Intake();
  private final Extender extender = new Extender();
  private final Grabber grabber = new Grabber();

  private final Command midAuto = new MidAuton(drive, intake);
  private final Command rightAuto = new RightAuton(drive, intake);
  private final Command coneAuto = new ConeAuton(extender, grabber);

  public static SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
    m_chooser.setDefaultOption("C", coneAuto);
    m_chooser.addOption("M", midAuto);
    m_chooser.addOption("R", rightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    rejectButton
        .onTrue(new InstantCommand(intake::reverseShooterMotors))
        .onFalse(new InstantCommand(intake::stopReversing));
    grabButton
        .onTrue(new InstantCommand(grabber::startGrabber))
        .onFalse(new InstantCommand(grabber::stopGrabber));
    dropButton
        .onTrue(new InstantCommand(grabber::dropCone))
        .onFalse(new InstantCommand(grabber::stopGrabber));
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.driveCommand(driveForwardAxis, driveLeftAxis, rotateAxis));
    extender.setDefaultCommand(extender.exCommand());
    intake.setDefaultCommand(intake.shootTeleOp());
  }

  public Command getAutonomousCommand() {

    return m_chooser.getSelected();

  }
}

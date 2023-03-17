// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.MidAuton;
import frc.robot.Commands.RightAuton;
//import frc.robot.Commands.ShooterIntakeControl;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.Joysticks.Axes.*;
import static frc.robot.Constants.Joysticks.Buttons.*;

public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  public static final Intake intake = new Intake();
  private final MidAuton auton = new MidAuton(drive,intake);
 
  private final RightAuton rAuton = new RightAuton(drive, intake);
  private  final Command kDefaultAuto = auton;
  private final Command kCustomAuto = rAuton;
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Extender extender = new Extender();
  //private final ShooterIntakeControl shooterIntakeControl = new ShooterIntakeControl(intake);
  
  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
    m_chooser.setDefaultOption("M", kDefaultAuto);
    m_chooser.addOption("R", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //shooterIntakeControl.repeatedly();
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
  
    extendButton
      .onTrue(new InstantCommand(extender::Extend))
      .onFalse(new InstantCommand(extender::Stop));
    retractButton
      .onTrue(new InstantCommand(extender::UnExtend))
      .onFalse(new InstantCommand(extender::Stop));
   rejectButton
    .onTrue(new InstantCommand(intake::reverseShooterMotors))
    .onFalse(new InstantCommand(intake::stopReversing));
   
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.driveCommand(driveForwardAxis, driveLeftAxis, rotateAxis));
    
  
  
  }

  public Command getAutonomousCommand() {
    
      return m_chooser.getSelected();
        
    }
  }


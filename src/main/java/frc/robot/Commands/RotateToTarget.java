// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sensors;
public class RotateToTarget extends CommandBase {
  private final DriveSubsystem m_driveSub;
  private boolean done = false;
  private double kP = 0.5;
  /** Creates a new RotateToTarget. */
  public RotateToTarget(DriveSubsystem driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSub = driveSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xDist = Sensors.x.get();
    PIDController pid = new PIDController(kP, 0, 0);
    if(xDist < 0.1 && xDist > -0.1){   
      m_driveSub.driveCommand(0, 0, 0, true);
      done = true;
        }else{
            if(xDist > 0){
              m_driveSub.driveCommand(0, 0, pid.calculate(xDist, 0), true);
            }else{
              m_driveSub.driveCommand(0, 0, pid.calculate(xDist, 0), true);
     }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

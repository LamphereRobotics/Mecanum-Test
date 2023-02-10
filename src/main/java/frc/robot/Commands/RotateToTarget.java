// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sensors;
public class RotateToTarget extends CommandBase {
  private final DriveSubsystem m_driveSub;
  private boolean done = false;
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
    if(xDist < 10 && xDist > -10){   
      m_driveSub.driveCommand(0, 0, 0, isFinished());
      done = true;
        }else{
            if(xDist > 0){
              m_driveSub.driveCommand(0, 0, -0.05, xDist > 0);
            }else{
              m_driveSub.driveCommand(0, 0, -0.05, xDist < 0);
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

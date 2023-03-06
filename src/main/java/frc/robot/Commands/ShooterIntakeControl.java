// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ShooterIntakeControl extends CommandBase {
  private final Intake m_Intake;
  /** Creates a new ShooterIntakeControl. */
  public ShooterIntakeControl(Intake intake) {
    m_Intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//     if(leftTriggerAxis.get() > 0.2){
//        m_Intake.startIntakeMotors();
//   }else{
    
//   }
//   if(leftTriggerAxis.get() < 0.2 && rightTriggerAxis.get() < 0.2){
//     m_Intake.stopIntakeMotors();
//   }
//   if(rightTriggerAxis.get() > 0.2){
//     m_Intake.startShooterMotors();
//     m_Intake.startIntakeMotors();
// }else{
//   m_Intake.stopShooterMotors();
// }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

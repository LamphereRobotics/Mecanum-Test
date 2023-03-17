// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAuton extends SequentialCommandGroup {
  private final DriveSubsystem m_driveSub;
  private final Intake m_int;
  /** Creates a new Auton. */
  public RightAuton(DriveSubsystem driveSub, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_int = intake;
    m_driveSub = driveSub;
    addRequirements(driveSub, intake);
   
    addCommands(

      m_driveSub.driveCommand(0, 0, 0, true).withTimeout(0.1),
      m_driveSub.driveCommand(0, 0, 0.2, true).withTimeout(0.1)
     
     );
    
  }
  
}

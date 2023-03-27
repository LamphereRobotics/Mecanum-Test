// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// Command intakeAndShoot = Commands.startEnd(() -> intake.set(1.0), () -> intake.set(0), intake)
//     .alongWith(new RunShooter(shooter));
import edu.wpi.first.wpilibj2.command.RunCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidAuton extends SequentialCommandGroup {
  private final DriveSubsystem m_driveSub;

  /** Creates a new Auton. */
  public MidAuton(DriveSubsystem driveSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    m_driveSub = driveSub;
    addCommands(

        m_driveSub.driveCommand(-0.5, 0, 0, false).withTimeout(8),
        m_driveSub.driveCommand(0, 0, 0, false).withTimeout(0.5)

    );

  }

}

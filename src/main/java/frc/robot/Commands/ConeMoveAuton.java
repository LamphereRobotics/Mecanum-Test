// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeMoveAuton extends SequentialCommandGroup {
  /** Creates a new ConeAuton. */
  public ConeMoveAuton(Extender extender, Grabber grabber, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConeStayAuton(grabber, extender, drive),
        drive.driveCommand(-0.5, 0, 0, false).withTimeout(8),
        drive.driveCommand(0, 0, 0, false).withTimeout(0.5),
        drive.invertCommand());
  }
}

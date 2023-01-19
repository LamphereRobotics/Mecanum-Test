package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Constants {
  public static class Joysticks {
    private static final Joystick leftDriveJoystick = new Joystick(0);
    private static final Joystick rightDriveJoystick = new Joystick(1);

    public static class Buttons {
      public static final JoystickButton toggleFieldRelativeButton = new JoystickButton(leftDriveJoystick, 1);
    }

    public static class Axes {
      public static final Supplier<Double> driveForwardAxis = () -> -leftDriveJoystick.getRawAxis(0);
      public static final Supplier<Double> driveLeftAxis = () -> -leftDriveJoystick.getRawAxis(1);
      public static final Supplier<Double> rotateAxis = () -> -rightDriveJoystick.getRawAxis(0);
    }
  }
}

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Constants {
  public static class Joysticks {
    // private static final Joystick leftDriveJoystick = new Joystick(0);
    // private static final Joystick rightDriveJoystick = new Joystick(1);
    public static final Joystick singleDriveJoystick = new Joystick(2);
    public static class Axes {
      public static final Supplier<Double> driveForwardAxis = () -> -singleDriveJoystick.getRawAxis(1);
      public static final Supplier<Double> driveLeftAxis = () -> -singleDriveJoystick.getRawAxis(0);
      public static final Supplier<Double> rotateAxis = () -> -singleDriveJoystick.getRawAxis(4);
    }
    public static class Buttons {
      public static final JoystickButton toggleFieldRelativeButton = new JoystickButton(singleDriveJoystick, 1);
      public static final JoystickButton minSpeedButton = new JoystickButton(singleDriveJoystick, 4);
      public static final JoystickButton lowSpeedButton = new JoystickButton(singleDriveJoystick, 5);
      public static final JoystickButton highSpeedButton = new JoystickButton(singleDriveJoystick, 6);
      public static final JoystickButton gyroResetButton = new JoystickButton(singleDriveJoystick, 8);
      public static final JoystickButton intakeButton = new JoystickButton(singleDriveJoystick, 10);
    }
  }
   

    public static class Motors{
      //Talons
      public static final int k_frontLeftMotor = 1;
      public static final int k_frontRightMotor = 2;
      public static final int k_rearLeftMotor = 3;
      public static final int k_rearRightMotor = 4;
      //Victors
      public static final int k_topIntakeMotor = 5;
      public static final int k_midIntakeMotor = 6;
      public static final int k_bottomIntakeMotor = 7;
      public static final int k_extenderMotor = 8;

    }

   
  }


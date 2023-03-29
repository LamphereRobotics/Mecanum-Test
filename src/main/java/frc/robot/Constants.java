package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {
  public static class Joysticks {
    public static final Joystick singleDriveJoystick = new Joystick(1);
    public static final Joystick operatorJoystick = new Joystick(0);

    public static class Axes {
      public static final Supplier<Double> driveForwardAxis = () -> -singleDriveJoystick.getRawAxis(1);
      public static final Supplier<Double> driveLeftAxis = () -> -singleDriveJoystick.getRawAxis(0);
      public static final Supplier<Double> leftTriggerAxis = () -> operatorJoystick.getRawAxis(2);
      public static final Supplier<Double> rightTriggerAxis = () -> operatorJoystick.getRawAxis(3);
      public static final Supplier<Double> rotateAxis = () -> -singleDriveJoystick.getRawAxis(4);
      public static final Supplier<Double> extendAxis = () -> operatorJoystick.getRawAxis(5);
    }

    public static class Buttons {
      // A Button to toggle field relative
      public static final JoystickButton toggleFieldRelativeButton = new JoystickButton(singleDriveJoystick, 1);
      // Y button for minimum speed
      public static final JoystickButton minSpeedButton = new JoystickButton(singleDriveJoystick, 4);
      // Left bumber for low speed
      public static final JoystickButton lowSpeedButton = new JoystickButton(singleDriveJoystick, 5);
      // Right bumper for high speed
      public static final JoystickButton highSpeedButton = new JoystickButton(singleDriveJoystick, 6);
      // left joystick click for gyro reset
      public static final JoystickButton gyroResetButton = new JoystickButton(singleDriveJoystick, 9);
      // left bumper to pick up cone
      public static final JoystickButton grabButton = new JoystickButton(operatorJoystick, 5);
      // right bumper to drop cone
      public static final JoystickButton dropButton = new JoystickButton(operatorJoystick, 6);
      // X button to reverse intake and shooters
      public static final Trigger rejectButton = new JoystickButton(operatorJoystick, 3);
      // Y Button to shoot High
      public static final JoystickButton highShootButton = new JoystickButton(operatorJoystick, 4);
      // A Button to shoot Low
      public static final JoystickButton lowShootButton = new JoystickButton(operatorJoystick, 2);

      public static final JoystickButton balanceButton = new JoystickButton(operatorJoystick, 7);
      
    }
  }

  public static class Motors {
    // Talons
    public static final int k_frontLeftMotor = 1;
    public static final int k_frontRightMotor = 2;
    public static final int k_rearLeftMotor = 3;
    public static final int k_rearRightMotor = 4;
    public static final int k_extenderMotor = 5;

    // Victors
    public static final int k_topIntakeMotor = 5;
    public static final int k_midIntakeMotor = 6;
    public static final int k_bottomIntakeMotor = 7;
  }

}

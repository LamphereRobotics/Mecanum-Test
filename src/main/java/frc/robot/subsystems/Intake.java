// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import static frc.robot.Constants.Joysticks.Axes.*;
import static frc.robot.Constants.Joysticks.Buttons.*;

public class Intake extends SubsystemBase {
  // private boolean reversing = false;
  WPI_VictorSPX topIntake = new WPI_VictorSPX(7);
  WPI_VictorSPX middleIntake = new WPI_VictorSPX(6);
  WPI_VictorSPX bottomIntake = new WPI_VictorSPX(5);

  /** Creates a new Intake. */
  public Intake() {
  }

  public void startIntakeMotors() {
    bottomIntake.set(ControlMode.PercentOutput, 0.75);
  }

  public void startShooterMotors(double top, double bottom) {
    topIntake.set(ControlMode.PercentOutput, -top);
    middleIntake.set(ControlMode.PercentOutput, -bottom);
  }

  public Command startShooterHigh() {
    return new RunCommand(() -> shooterHigh(), this);
  }

  public Command startShooterLow() {
    return new RunCommand(() -> shooterLow(), this);//0.3, 0.4
  }
  public void shooterHigh() {
    startShooterMotors(0.55, 0.55);
  }

  public void shooterLow() {
    startShooterMotors(0.45, 0.45);//0.3, 0.4
  }

  public void reverseShooterMotors() {
    topIntake.set(ControlMode.PercentOutput, 0.8);
    middleIntake.set(ControlMode.PercentOutput, 0.7);
    bottomIntake.set(ControlMode.PercentOutput, -0.5);
  }

  public void stopReversing() {

  }

  public void stopIntakeMotors() {
    bottomIntake.set(ControlMode.PercentOutput, 0);
  }

  public void stopShooterMotors() {
    topIntake.set(ControlMode.PercentOutput, 0);
    middleIntake.set(ControlMode.PercentOutput, 0);
  }

  private void periodicCode() {
    if (!rejectButton.getAsBoolean()) {
      if (leftTriggerAxis.get() > 0.2) {
        startIntakeMotors();
      }
      if (leftTriggerAxis.get() < 0.2 && rightTriggerAxis.get() < 0.2) {
        stopIntakeMotors();
      }
      if (lowShootButton.getAsBoolean()) {
        shooterLow();
        startIntakeMotors();
      } else if (highShootButton.getAsBoolean()) {
        shooterHigh();
        startIntakeMotors();
      } else {
        stopShooterMotors();
      }
    } else {
      reverseShooterMotors();
    }
  }

  public Command shootTeleOp() {
    return new RunCommand(() -> periodicCode(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

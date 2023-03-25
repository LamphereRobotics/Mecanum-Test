// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.Motors;
import static frc.robot.Constants.Joysticks.Axes.*;

public class Extender extends SubsystemBase {
  private WPI_TalonFX extender = new WPI_TalonFX(Motors.k_extenderMotor);
  
  public double position = extender.getSelectedSensorPosition();

  /** Creates a new Extender. */

  public Extender() {
    extender.setNeutralMode(NeutralMode.Brake);
  }

  public void Extend() {
    // Stop the motor when fully extended.
    if (position >= 200000) {
      extender.stopMotor();
    }
    extender.set(ControlMode.PercentOutput, 0.5);
  }

  public void UnExtend() {
    // Stop the motor when fully retracted.
    if (position <= 0) {
      extender.stopMotor();
    }
    extender.set(ControlMode.PercentOutput, -0.5);
  }

  public void Stop() {
    extender.set(ControlMode.PercentOutput, 0);
  }

  public void exAxis() {
    if (Math.abs(extendAxis.get()) > 0.1) {
      extender.set(ControlMode.PercentOutput, extendAxis.get());
    } else {
      extender.set(ControlMode.PercentOutput, 0);
    }
  }

  public Command exCommand() {
    return new RunCommand(
        () -> exAxis(), this);
  }

  @Override
  public void periodic() {
    // Update the arm position on every loop.
    position = extender.getSelectedSensorPosition();
  }
}

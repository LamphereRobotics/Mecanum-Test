// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.Motors;
import static frc.robot.Constants.Joysticks.Axes.*;

public class Extender extends SubsystemBase {
  private WPI_TalonFX extender = new WPI_TalonFX(Motors.k_extenderMotor);

  public double position = extender.getSelectedSensorPosition();

  private static final double fullExtend = 170000;
  private static final double fullRetract = 10000;

  /** Creates a new Extender. */

  public Extender() {
    extender.setNeutralMode(NeutralMode.Brake);
    extender.setInverted(InvertType.InvertMotorOutput);
    // extender.setSensorPhase(true);
  }

  private void Extend() {
    // Stop the motor when fully extended.
    if (isFullyExtended()) {
      extender.stopMotor();
    } else {
      extender.set(ControlMode.PercentOutput, 1);
    }
  }

  private void UnExtend() {
    // Stop the motor when fully retracted.
    if (isFullyRetracted()) {
      extender.stopMotor();
    } else {
      extender.set(ControlMode.PercentOutput, -0.5);
    }
  }

  public Command ExtendCommand() {
    return new RunCommand(this::Extend, this)
        .until(this::isFullyExtended)
        .andThen(new InstantCommand(this::Stop, this));
  }

  public Command RetractCommand() {
    return new RunCommand(this::UnExtend, this)
        .until(this::isFullyRetracted)
        .andThen(new InstantCommand(this::Stop, this));
  }

  public void Stop() {
    extender.set(ControlMode.PercentOutput, 0);
  }

  public void exAxis() {
    double out = -extendAxis.get();

    // Reduce speed retracting
    if (out < 0)
      out *= 0.6;

    // Deadband
    if (Math.abs(out) < 0.1) {
      out = -0.6;// -0.7
    }

    // extension limits
    if ((out > 0 && isFullyExtended())
        || (out < 0 && isFullyRetracted())) {
      out = 0;
    }

    extender.set(ControlMode.PercentOutput, out);
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

  public boolean isFullyExtended() {
    return position >= fullExtend;
  }

  public boolean isFullyRetracted() {
    return position <= fullRetract;
  }
}

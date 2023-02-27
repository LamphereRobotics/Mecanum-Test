// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
public class Intake extends SubsystemBase {
  WPI_VictorSPX topIntake = new WPI_VictorSPX(7);
  WPI_VictorSPX middleIntake = new WPI_VictorSPX(6);
  WPI_VictorSPX bottomIntake = new WPI_VictorSPX(5);
  /** Creates a new Intake. */
  public Intake() {}

  public void startIntakeMotors(){
    bottomIntake.set(ControlMode.PercentOutput, 0.75);
  }

  public void startShooterMotors(){
    topIntake.set(ControlMode.PercentOutput, 0.75);
    middleIntake.set(ControlMode.PercentOutput, 0.65);
  }

  public void stopIntakeMotor(){
    bottomIntake.set(ControlMode.PercentOutput, 0);
  }
  
  public void stopMotors(){
    topIntake.set(ControlMode.PercentOutput, 0);
    middleIntake.set(ControlMode.PercentOutput, 0);
    bottomIntake.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

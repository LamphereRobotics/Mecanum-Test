// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
public class Grabber extends SubsystemBase {
  WPI_VictorSPX grabber = new WPI_VictorSPX(10);
  /** Creates a new Grabber. */
  public Grabber() {}
public void startGrabber(){
  grabber.set(ControlMode.PercentOutput, 1);
}
public void dropCone(){
  grabber.set(ControlMode.PercentOutput, -1);
}
public void stopGrabber(){
  grabber.set(ControlMode.PercentOutput, 0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

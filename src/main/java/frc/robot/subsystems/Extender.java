// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.Motors;
public class Extender extends SubsystemBase {
  WPI_VictorSPX extender = new WPI_VictorSPX(Motors.k_extenderMotor);
  /** Creates a new Extender. */
  
  public Extender() {}
  public void Grab(){
    
  }
  public void UnGrab(){

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

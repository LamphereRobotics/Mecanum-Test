// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//import edu.wpi.first.math.Vector;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Vector;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class Sensors extends SubsystemBase {
  
   
   static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry tx = table.getEntry("tx");
  public static NetworkTableEntry ty = table.getEntry("ty");
  public static NetworkTableEntry ta = table.getEntry("ta");
  public static Supplier<Double> x = () -> tx.getDouble(0.0);
  public static  Supplier<Double> y =() -> ty.getDouble(0.0);
    Supplier<Double> area =() -> ta.getDouble(0.0);
  /** Creates a new Sensors. */
  public Sensors() {
  
    
    //read values periodically
    
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x.get());
    SmartDashboard.putNumber("LimelightY", y.get());
    SmartDashboard.putNumber("LimelightArea", area.get());
    
 

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 
}


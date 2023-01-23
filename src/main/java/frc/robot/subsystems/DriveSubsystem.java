// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  private static final double kMinSpeed = 0.05; // 0.05 meters per second
  private static final double kMaxSpeed = 8.0; // 8 meters per second
  private static final double kMaxRotationSpeed = Math.PI; // 1/2 rotation per second

  private static final double kTalonUnitsPerRotation = 2048.0;
  private static final double kGearRatio = 13.0 / 72.0; // 13 wheel rotations per 72 motor rotations
  private static final double kWheelDiameter = 0.1524; // 6 inches = 0.1524 meters
  // meters to talon units
  private static final double kTalonUnitsToMetersMultiplier = (1 / kTalonUnitsPerRotation) // motor rotations
      * kGearRatio // wheel rotations
      * Math.PI * kWheelDiameter; // distance

  // TODO: tune this because it wont even be close anymore.
  private static final double kP = 5; // An error of 1 rotation per second results in 5 amps output
  private static final double kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
  private static final double kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  private static final double kF = 0.04727; // Feed forward for velocity
  private static final double kS = 1; // Arbitrary feed forward to overcome static friction

  // Peak output of 40 amps
  private static final double kPeakCurrent = 40.0;

  private final WPI_TalonFX frontLeftMotor = new WPI_TalonFX(1);
  private final WPI_TalonFX frontRightMotor = new WPI_TalonFX(2);
  private final WPI_TalonFX rearLeftMotor = new WPI_TalonFX(3);
  private final WPI_TalonFX rearRightMotor = new WPI_TalonFX(4);

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d rearLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d rearRightLocation = new Translation2d(-0.381, -0.381);

  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
      frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);

  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(0);

  private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, gyro.getRotation2d(),
      getCurrentPositions());

  private boolean fieldRelative = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.reset();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /*
     * Torque-based velocity does not require a feed forward, as torque will
     * accelerate the rotor up to the desired velocity by itself
     */
    configs.slot0.kP = kP;
    configs.slot0.kI = kI;
    configs.slot0.kD = kD;
    configs.slot0.kF = kF;

    configs.initializationStrategy = SensorInitializationStrategy.BootToZero;
    configs.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, kPeakCurrent, kPeakCurrent, 0);
    configs.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

    frontLeftMotor.configAllSettings(configs);
    frontRightMotor.configAllSettings(configs);
    rearLeftMotor.configAllSettings(configs);
    rearRightMotor.configAllSettings(configs);

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }

  public MecanumDriveWheelSpeeds getCurrentSpeeds() {
    return new MecanumDriveWheelSpeeds(
        getMotorSpeed(frontLeftMotor),
        getMotorSpeed(frontRightMotor),
        getMotorSpeed(rearLeftMotor),
        getMotorSpeed(rearRightMotor));
  }

  private double getMotorSpeed(WPI_TalonFX motor) {
    return talonUnitsToMeters(rearRightMotor.getSelectedSensorVelocity()) * 10;
  }

  public MecanumDriveWheelPositions getCurrentPositions() {
    return new MecanumDriveWheelPositions(
        getMotorPosition(frontLeftMotor),
        getMotorPosition(frontRightMotor),
        getMotorPosition(rearLeftMotor),
        getMotorPosition(rearRightMotor));
  }

  private double getMotorPosition(WPI_TalonFX motor) {
    return talonUnitsToMeters(rearRightMotor.getSelectedSensorPosition());
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    setMotorSpeed(frontLeftMotor, speeds.frontLeftMetersPerSecond);
    setMotorSpeed(frontRightMotor, speeds.frontRightMetersPerSecond);
    setMotorSpeed(rearLeftMotor, speeds.rearLeftMetersPerSecond);
    setMotorSpeed(rearRightMotor, speeds.rearRightMetersPerSecond);
  }

  private void setMotorSpeed(WPI_TalonFX motor, double speed) {
    if (Math.abs(speed) < kMinSpeed) {
      speed = 0;
    }
    motor.set(TalonFXControlMode.Velocity, metersToTalonUnits(speed) / 10, DemandType.ArbitraryFeedForward,
        Math.signum(speed) * kS);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  public void driveTeleop(double x, double y, double z) {
    x *= kMaxSpeed;
    y *= kMaxSpeed;
    z *= kMaxRotationSpeed;

    if (fieldRelative) {
      driveFieldRelative(x, y, z);
    } else {
      driveRobotRelative(x, y, z);
    }
  }

  private void driveRobotRelative(double x, double y, double z) {
    drive(new ChassisSpeeds(x, y, z));
  }

  private void driveFieldRelative(double x, double y, double z) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, gyro.getRotation2d()));
  }

  private void setFieldRelative(boolean value) {
    fieldRelative = value;
  }

  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), getCurrentPositions());
  }

  public Command driveCommand(double x, double y, double z, boolean fieldRelative) {
    return new RunCommand(
        fieldRelative ? () -> driveFieldRelative(x, y, z) : () -> driveRobotRelative(x, y, z), this);
  }

  public Command driveCommand(Supplier<Double> x, Supplier<Double> y, Supplier<Double> z) {
    return new RunCommand(() -> driveTeleop(x.get(), y.get(), z.get()), this);
  }

  public Command setFieldRelativeCommand(boolean value) {
    return new InstantCommand(() -> setFieldRelative(value), this);
  }

  private static double metersToTalonUnits(double meters) {
    return meters / kTalonUnitsToMetersMultiplier;
  }

  private static double talonUnitsToMeters(double talonUnits) {
    return talonUnits * kTalonUnitsToMetersMultiplier;
  }
}

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

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class DriveSubsystem extends SubsystemBase {
  private static final double kMinSpeed = 0.05; // 0.05 meters per second
  private static final double kMaxSpeed = 8.0; // 8 meters per second
  private static final double kMaxRotation = Math.PI; // 1/2 rotation per second
  private static final double kWheelDiameter = 0.1524; // 6 inches = 0.1524 meters
  private static final double kGearRatio = 72 / 13; // 72 motor rotations per wheel rotation
  // meters per second to motor rotations per second
  private static final double kSpeedToRotationsMultiplier = (1 / (Math.PI * kWheelDiameter)) * kGearRatio;

  private static final double kP = 5; // An error of 1 rotation per second results in 5 amps output
  private static final double kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
  private static final double kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  private static final double kF = 1; // Feed forward to overcome static friction
  private static final double kOutputRampPeriod = 0; // Seconds to go from 0A to +-40A output

  // Peak output of 40 amps
  private static final double kPeakCurrent = 40;

  private final TalonFX frontLeftMotor = new TalonFX(1);
  private final TalonFX frontRightMotor = new TalonFX(2);
  private final TalonFX rearLeftMotor = new TalonFX(3);
  private final TalonFX rearRightMotor = new TalonFX(4);

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d rearLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d rearRightLocation = new Translation2d(-0.381, -0.381);

  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
      frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);

  private final Pigeon2 gyro = new Pigeon2(0);

  private final Supplier<Double> frontLeftVelocity = frontLeftMotor.getVelocity().asSupplier();
  private final Supplier<Double> frontRightVelocity = frontRightMotor.getVelocity().asSupplier();
  private final Supplier<Double> rearLeftVelocity = rearLeftMotor.getVelocity().asSupplier();
  private final Supplier<Double> rearRightVelocity = rearRightMotor.getVelocity().asSupplier();

  private final Supplier<Double> frontLeftPosition = frontLeftMotor.getPosition().asSupplier();
  private final Supplier<Double> frontRightPosition = frontRightMotor.getPosition().asSupplier();
  private final Supplier<Double> rearLeftPosition = rearLeftMotor.getPosition().asSupplier();
  private final Supplier<Double> rearRightPosition = rearRightMotor.getPosition().asSupplier();

  private final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, false);

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
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakCurrent;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = (300 / kPeakCurrent) * kOutputRampPeriod;

    frontLeftMotor.getConfigurator().apply(configs);
    frontRightMotor.getConfigurator().apply(configs);
    rearLeftMotor.getConfigurator().apply(configs);
    rearRightMotor.getConfigurator().apply(configs);

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        frontLeftVelocity.get(),
        frontRightVelocity.get(),
        rearLeftVelocity.get(),
        rearRightVelocity.get());
  }

  public MecanumDriveWheelPositions getCurrentPositions() {
    return new MecanumDriveWheelPositions(
        frontLeftPosition.get(),
        frontRightPosition.get(),
        rearLeftPosition.get(),
        rearRightPosition.get());
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    setMotorVelocity(frontLeftMotor, speeds.frontLeftMetersPerSecond);
    setMotorVelocity(frontRightMotor, speeds.frontRightMetersPerSecond);
    setMotorVelocity(rearLeftMotor, speeds.rearLeftMetersPerSecond);
    setMotorVelocity(rearRightMotor, speeds.rearRightMetersPerSecond);
  }

  private void setMotorVelocity(TalonFX motor, double speed) {
    double feedForward = 0;
    if (Math.abs(speed) < kMinSpeed) {
      speed = 0;
    } else if (speed > 0) {
      feedForward = kF;
    } else {
      feedForward = -kF;
    }
    motor
        .setControl(torqueVelocity
            .withVelocity(speed * kSpeedToRotationsMultiplier)
            .withFeedForward(feedForward));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  public void driveTeleop(double x, double y, double z) {
    x *= kMaxSpeed;
    y *= kMaxSpeed;
    z *= kMaxRotation;

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
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Units;

public class Drivetrain extends SubsystemBase {

  private static CANSparkMax m_leftDriveOne;  
  private static CANSparkMax m_leftDriveTwo;
  private static CANSparkMax m_leftDriveThree;

  private static CANSparkMax m_rightDriveOne;  
  private static CANSparkMax m_rightDriveTwo;
  private static CANSparkMax m_rightDriveThree;

  private static RelativeEncoder m_leftEncoder;
  private static RelativeEncoder m_rightEncoder;

  private static Pigeon2 m_gyro;

  private double m_speedInvert = 1.0;
  private double m_rotationInvert = 1.0;
  private double m_multiplier = 1.0;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_leftDriveOne = new CANSparkMax(Constants.Drive.kLeftMotorOneId, MotorType.kBrushless);
    m_leftDriveTwo = new CANSparkMax(Constants.Drive.kLeftMotorTwoId, MotorType.kBrushless);
    m_leftDriveThree = new CANSparkMax(Constants.Drive.kLeftMotorThreeId, MotorType.kBrushless);

    m_rightDriveOne = new CANSparkMax(Constants.Drive.kRightMotorOneId, MotorType.kBrushless);
    m_rightDriveTwo = new CANSparkMax(Constants.Drive.kRightMotorTwoId, MotorType.kBrushless);
    m_rightDriveThree = new CANSparkMax(Constants.Drive.kRightMotorThreeId, MotorType.kBrushless);

    m_leftDriveOne.restoreFactoryDefaults();
    m_leftDriveTwo.restoreFactoryDefaults();
    m_leftDriveThree.restoreFactoryDefaults();

    m_rightDriveOne.restoreFactoryDefaults();
    m_rightDriveTwo.restoreFactoryDefaults();
    m_rightDriveThree.restoreFactoryDefaults();

    m_leftDriveTwo.follow(m_leftDriveOne);
    m_leftDriveThree.follow(m_leftDriveOne);

    m_rightDriveTwo.follow(m_rightDriveOne);
    m_rightDriveThree.follow(m_rightDriveOne);

    m_leftDriveOne.setInverted(false);
    m_leftDriveTwo.setInverted(false);
    m_leftDriveThree.setInverted(false);

    m_rightDriveOne.setInverted(true);
    m_rightDriveTwo.setInverted(true);
    m_rightDriveThree.setInverted(true);

    m_leftDriveOne.setOpenLoopRampRate(Constants.Drive.kRampRate);
    m_leftDriveTwo.setOpenLoopRampRate(Constants.Drive.kRampRate);
    m_leftDriveThree.setOpenLoopRampRate(Constants.Drive.kRampRate);

    m_rightDriveOne.setOpenLoopRampRate(Constants.Drive.kRampRate);
    m_rightDriveTwo.setOpenLoopRampRate(Constants.Drive.kRampRate);
    m_rightDriveThree.setOpenLoopRampRate(Constants.Drive.kRampRate);

    m_leftDriveOne.setIdleMode(IdleMode.kBrake);
    m_leftDriveTwo.setIdleMode(IdleMode.kBrake);
    m_leftDriveThree.setIdleMode(IdleMode.kBrake);

    m_rightDriveOne.setIdleMode(IdleMode.kBrake);
    m_rightDriveTwo.setIdleMode(IdleMode.kBrake);
    m_rightDriveThree.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftDriveOne.getEncoder();
    m_rightEncoder = m_rightDriveOne.getEncoder();

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_gyro = new Pigeon2(Constants.Sensors.kDrivetrainGyroId);
  }

  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drivetrain");
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Roll", this::getRoll, null);
    builder.addDoubleProperty("Left Encoder Position", this::getLeftPosition, null);
    builder.addDoubleProperty("Right Encoder Position", this::getRightPosition, null);
    builder.addDoubleProperty("Left Encoder Velocity", this::getLeftVelocity, null);
    builder.addDoubleProperty("Right Encoder Velocity", this::getRightVelocity, null);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {

    double leftInput = Math.abs(leftSpeed) > 0.1 ? leftSpeed : 0;
    double rightInput = Math.abs(rightSpeed) > 0.1 ? rightSpeed : 0;

    m_leftDriveOne.set(leftInput * m_speedInvert * m_multiplier);
    m_rightDriveOne.set(rightInput * m_speedInvert * m_multiplier);
  }

  public void arcadeDrive(double speed, double rotation) {

    double speedInput = Math.abs(speed) > 0.1 ? speed : 0;
    double rotationInput = Math.abs(rotation) > 0.1 ? rotation : 0;

    m_leftDriveOne.set((speedInput * m_speedInvert + rotationInput * m_rotationInvert) * m_multiplier);
    m_rightDriveOne.set((speedInput * m_speedInvert - rotationInput * m_rotationInvert) * m_multiplier);
  }

  public void invertSpeed() {
    m_speedInvert = m_speedInvert > 0.0 ? -1.0 : 1.0;
  }

  public void invertRotation() {
    m_rotationInvert = m_rotationInvert > 0.0 ? -1.0 : 1.0;
  }

  public void setMultiplier(double multiplier) {
    m_multiplier = multiplier;
  }

  public void stop() {
    m_leftDriveOne.set(0);
    m_rightDriveOne.set(0);
  }

  public double getYaw() { 
    return m_gyro.getYaw(); 
  }

  public double getPitch() { 
    return m_gyro.getPitch(); 
  }

  public double getRoll() { 
    return m_gyro.getRoll(); 
  }

  public double getLeftPosition() {
    return ((m_leftEncoder.getPosition() * Constants.Drive.kMotorToWheel) / Constants.Drive.kEncoderResolution) * Units.inchesToMeters(Constants.Drive.kWheelDiameter) * Math.PI;
  }

  public double getRightPosition() {
    return (m_rightEncoder.getPosition() * Constants.Drive.kMotorToWheel) / Constants.Drive.kEncoderResolution * Units.inchesToMeters(Constants.Drive.kWheelDiameter) * Math.PI;
  }

  public double getLeftVelocity() {
    return (m_leftEncoder.getVelocity() / 60.0) * Units.inchesToMeters(Constants.Drive.kWheelDiameter) * Math.PI;
  }

  public double getRightVelocity() {
    return (m_rightEncoder.getVelocity() / 60.0) * Units.inchesToMeters(Constants.Drive.kWheelDiameter) * Math.PI;
  }

  public double getAverageDistance() {
    return (getLeftPosition() + getRightPosition()) / 2;
  }

  public boolean isTipped() {
    return Math.abs(m_gyro.getRoll()) > 13.0;
  }

  public boolean isLevel() {
    return Math.abs(m_gyro.getRoll()) < 5.0;
  }
}


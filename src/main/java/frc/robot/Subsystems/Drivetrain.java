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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static CANSparkMax m_leftDriveOne;  
  private static CANSparkMax m_leftDriveTwo;
  private static CANSparkMax m_leftDriveThree;

  private static CANSparkMax m_rightDriveOne;  
  private static CANSparkMax m_rightDriveTwo;
  private static CANSparkMax m_rightDriveThree;

  private static RelativeEncoder m_leftEncoder;
  private static RelativeEncoder m_rightEncoder;

  private static DifferentialDrive m_drive;

  private static Pigeon2 m_gyro;

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

    m_leftDriveOne.setIdleMode(IdleMode.kBrake);
    m_leftDriveTwo.setIdleMode(IdleMode.kBrake);
    m_leftDriveThree.setIdleMode(IdleMode.kBrake);

    m_rightDriveOne.setIdleMode(IdleMode.kBrake);
    m_rightDriveTwo.setIdleMode(IdleMode.kBrake);
    m_rightDriveThree.setIdleMode(IdleMode.kBrake);

    m_leftEncoder = m_leftDriveOne.getEncoder();
    m_rightEncoder = m_rightDriveOne.getEncoder();

    m_drive = new DifferentialDrive(m_leftDriveOne, m_rightDriveOne);

    m_gyro = new Pigeon2(Constants.Sensors.kDrivetrainGyroId);
  }

  @Override
  public void periodic() {
    m_drive.feed();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drivetrain");
    builder.addDoubleProperty("Drivetrain Yaw", this::getYaw, null);
    builder.addDoubleProperty("Drivetrain Pitch", this::getPitch, null);
    builder.addDoubleProperty("Drivetrain Roll", this::getRoll, null);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void stop() {
    m_leftDriveOne.set(0);
    m_rightDriveOne.set(0);
  }

  public void zero() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getYaw() { return m_gyro.getYaw(); }

  public double getPitch() { return m_gyro.getPitch(); }

  public double getRoll() { return m_gyro.getRoll(); }
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static CANSparkMax m_leftDriveOne;  
  private static CANSparkMax m_leftDriveTwo;
  private static CANSparkMax m_leftDriveThree;

  private static CANSparkMax m_rightDriveOne;  
  private static CANSparkMax m_rightDriveTwo;
  private static CANSparkMax m_rightDriveThree;

  private static Pigeon2 m_gyro;

  private static NetworkTableInstance tableInstance;
  private static NetworkTable gyroTable;
  private static NetworkTableEntry gyroYaw;
  private static NetworkTableEntry gyroPitch;
  private static NetworkTableEntry gyroRoll;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
      m_leftDriveOne = new CANSparkMax(Constants.Drive.kLeftMotorOneId, MotorType.kBrushless);
      m_leftDriveTwo = new CANSparkMax(Constants.Drive.kLeftMotorTwoId, MotorType.kBrushless);
      m_leftDriveThree = new CANSparkMax(Constants.Drive.kLeftMotorThreeId, MotorType.kBrushless);

      m_rightDriveOne = new CANSparkMax(Constants.Drive.kRightMotorOneId, MotorType.kBrushless);
      m_rightDriveTwo = new CANSparkMax(Constants.Drive.kRightMotorTwoId, MotorType.kBrushless);
      m_rightDriveThree = new CANSparkMax(Constants.Drive.kRightMotorThreeId, MotorType.kBrushless);

      m_gyro = new Pigeon2(Constants.Sensors.kPigeonId);

      tableInstance = NetworkTableInstance.getDefault();
      gyroTable = tableInstance.getTable("Drivetrain Gyro");
      gyroYaw = gyroTable.getEntry("Yaw");
      gyroPitch = gyroTable.getEntry("Pitch");
      gyroRoll = gyroTable.getEntry("Roll");

      m_leftDriveTwo.follow(m_leftDriveOne);
      m_leftDriveThree.follow(m_leftDriveOne);

      m_rightDriveTwo.follow(m_rightDriveOne);
      m_rightDriveThree.follow(m_rightDriveOne);

      m_rightDriveOne.setInverted(true);
      m_rightDriveTwo.setInverted(true);
      m_rightDriveThree.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gyroYaw.setDouble(m_gyro.getYaw());
    gyroPitch.setDouble(m_gyro.getPitch());
    gyroRoll.setDouble(m_gyro.getRoll());
  }

  public void drive(double leftSpeed, double rightSpeed) {
    m_leftDriveOne.set(leftSpeed);
    m_rightDriveOne.set(rightSpeed);
  }

  public void stop() {
    m_leftDriveOne.set(0);
    m_rightDriveOne.set(0);
  }

  public void enableBreak() {
    m_leftDriveOne.setIdleMode(IdleMode.kBrake);
    m_leftDriveTwo.setIdleMode(IdleMode.kBrake);
    m_leftDriveThree.setIdleMode(IdleMode.kBrake);

    m_rightDriveOne.setIdleMode(IdleMode.kBrake);
    m_rightDriveTwo.setIdleMode(IdleMode.kBrake);
    m_rightDriveThree.setIdleMode(IdleMode.kBrake);
  }

  public void disableBreak() {
    m_leftDriveOne.setIdleMode(IdleMode.kCoast);
    m_leftDriveTwo.setIdleMode(IdleMode.kCoast);
    m_leftDriveThree.setIdleMode(IdleMode.kCoast);

    m_rightDriveOne.setIdleMode(IdleMode.kCoast);
    m_rightDriveTwo.setIdleMode(IdleMode.kCoast);
    m_rightDriveThree.setIdleMode(IdleMode.kCoast);
  }

  public Pigeon2 getGyro() { return m_gyro; }
}


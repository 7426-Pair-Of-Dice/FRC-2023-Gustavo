// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Units;

public class Wrist extends SubsystemBase {

  private static TalonFX m_wristMotor;

  private static Pigeon2 m_gyro;

  private static double m_setpoint;

  /** Creates a new Wrist. */
  public Wrist() {
    m_wristMotor = new TalonFX(Constants.Wrist.kWristMotorId);

    m_gyro = new Pigeon2(Constants.Sensors.kClawGyroId);

    m_wristMotor.configFactoryDefault();

    m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    m_wristMotor.setSelectedSensorPosition(0);

    m_wristMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.selectProfileSlot(0, 0);
    m_wristMotor.config_kF(0, 0.2);
    m_wristMotor.config_kP(0, 0.08);
    m_wristMotor.config_kI(0, 0);
    m_wristMotor.config_kD(0,0);

    m_wristMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(120, Constants.Wrist.kMotorToWrist, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configReverseSoftLimitThreshold(0, Constants.TalonFX.kTimeoutMs);
    
    m_wristMotor.configForwardSoftLimitEnable(true);
    m_wristMotor.configReverseSoftLimitEnable(true);

    m_wristMotor.configMotionCruiseVelocity(20000, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configMotionAcceleration(15000, Constants.TalonFX.kTimeoutMs); 

    m_wristMotor.configNeutralDeadband(0.05);

    m_wristMotor.setNeutralMode(NeutralMode.Brake);

    m_wristMotor.setInverted(true);

    m_setpoint = getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Wrist");
    builder.addDoubleProperty("Wrist Yaw", this::getYaw, null);
    builder.addDoubleProperty("Wrist Pitch", this::getPitch, null);
    builder.addDoubleProperty("Wrist Roll", this::getRoll, null);
    builder.addDoubleProperty("Wrist Position", this::getPosition, null);
    builder.addDoubleProperty("Wrist Angle", this::getAngle, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_wristMotor.set(ControlMode.PercentOutput, percentOutput);
    m_setpoint = getPosition();
  }

  public void setPosition(double degrees) {
    double ticks = Units.degreesToTicks(degrees, Constants.Wrist.kMotorToWrist, Constants.TalonFX.kEncoderResolution);
    m_setpoint = ticks;
    m_wristMotor.set(ControlMode.MotionMagic, ticks);
  }

  public void setLastPosition() {
    m_wristMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public double getPosition() {
    return m_wristMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    return Units.ticksToDegrees(getPosition(), Constants.Wrist.kMotorToWrist, Constants.TalonFX.kEncoderResolution);
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

  public boolean atSetpoint() {
    return Math.abs(m_setpoint - getPosition()) < 5.0;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Units;

public class Shoulder extends SubsystemBase {

  private static TalonFX m_shoulderMotor;
  private static TalonFX m_shoulderMotorFollower;

  private static double m_setpoint;

  /** Creates a new Arm. */
  public Shoulder() {

    m_shoulderMotor = new TalonFX(Constants.Shoulder.kLeftArmMotorId);
    m_shoulderMotorFollower = new TalonFX(Constants.Shoulder.kRightArmMotorId);

    // Arm Configuration
    m_shoulderMotor.configFactoryDefault();
    m_shoulderMotorFollower.configFactoryDefault();

    m_shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_shoulderMotor.setSelectedSensorPosition(0);

    m_shoulderMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_shoulderMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);

    m_shoulderMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(100, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configReverseSoftLimitThreshold(Units.degreesToTicks(15, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configForwardSoftLimitEnable(true);
    m_shoulderMotor.configReverseSoftLimitEnable(true);

    m_shoulderMotor.selectProfileSlot(0, 0);
    m_shoulderMotor.config_kF(0, 0.2);
    m_shoulderMotor.config_kP(0, 0.08);
    m_shoulderMotor.config_kI(0, 0.0);
    m_shoulderMotor.config_kD(0, 0.0);

    m_shoulderMotor.configMotionCruiseVelocity(10000, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configMotionAcceleration(5000, Constants.TalonFX.kTimeoutMs);

    m_shoulderMotor.configNeutralDeadband(0.05);
    m_shoulderMotorFollower.configNeutralDeadband(0.05);

    m_shoulderMotorFollower.follow(m_shoulderMotor);
    m_shoulderMotorFollower.setInverted(TalonFXInvertType.OpposeMaster);

    m_shoulderMotor.configOpenloopRamp(0.25);

    m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
    m_shoulderMotorFollower.setNeutralMode(NeutralMode.Brake);

    m_setpoint = Units.degreesToTicks(15, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Arm");
      builder.addDoubleProperty("Arm Position", this::getPosition, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_shoulderMotor.set(ControlMode.PercentOutput, percentOutput);
    m_setpoint = getPosition();
  }

  public void setPosition(double degrees) {
    double ticks = Units.degreesToTicks(degrees, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution);
    m_setpoint = ticks;
    m_shoulderMotor.set(ControlMode.MotionMagic, ticks);
  }

  public void setLastPosition() {
    m_shoulderMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public void stop() {
    m_shoulderMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getPosition() {
    return m_shoulderMotor.getSelectedSensorPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - m_setpoint) < 3.0;
  }
}

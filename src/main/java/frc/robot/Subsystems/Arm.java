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

public class Arm extends SubsystemBase {

  private static TalonFX m_armMotor;
  private static TalonFX m_armMotorFollower;

  private static double m_lastArmPosition;

  /** Creates a new Arm. */
  public Arm() {

    m_armMotor = new TalonFX(Constants.Arm.kLeftArmMotorId);
    m_armMotorFollower = new TalonFX(Constants.Arm.kRightArmMotorId);

    // Arm Configuration
    m_armMotor.configFactoryDefault();
    m_armMotorFollower.configFactoryDefault();

    m_armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_armMotor.setSelectedSensorPosition(0);

    m_armMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_armMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);

    m_armMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(100, Constants.Arm.kMotorToArm, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_armMotor.configReverseSoftLimitThreshold(0, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configForwardSoftLimitEnable(true);
    m_armMotor.configReverseSoftLimitEnable(true);

    m_armMotor.selectProfileSlot(0, 0);
    m_armMotor.config_kF(0, 0.2);
    m_armMotor.config_kP(0, 0.08);
    m_armMotor.config_kI(0, 0.0);
    m_armMotor.config_kD(0, 0.0);

    m_armMotor.configMotionCruiseVelocity(10000, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configMotionAcceleration(5000, Constants.TalonFX.kTimeoutMs);

    m_armMotor.configNeutralDeadband(0.05);
    m_armMotorFollower.configNeutralDeadband(0.05);

    m_armMotorFollower.follow(m_armMotor);
    m_armMotorFollower.setInverted(TalonFXInvertType.OpposeMaster);

    m_armMotor.setNeutralMode(NeutralMode.Brake);
    m_armMotorFollower.setNeutralMode(NeutralMode.Brake);
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
    m_armMotor.set(ControlMode.PercentOutput, percentOutput);
    m_lastArmPosition = getPosition();
  }

  public void setPosition(double degrees) {
    double ticks = Units.degreesToTicks(degrees, Constants.Arm.kMotorToArm, Constants.TalonFX.kEncoderResolution);
    m_lastArmPosition = ticks;
    m_armMotor.set(ControlMode.MotionMagic, ticks);
  }

  public void setLastPosition() {
    m_armMotor.set(ControlMode.MotionMagic, m_lastArmPosition);
  }

  public void stop() {
    m_armMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getPosition() {
    return m_armMotor.getSelectedSensorPosition();
  }
}

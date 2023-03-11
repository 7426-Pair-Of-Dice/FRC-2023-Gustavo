// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Units;

public class Shoulder extends SubsystemBase {

  private static TalonFX m_shoulderMotor;
  private static TalonFX m_shoulderMotorFollower;

  /* private static TalonSRX m_shoulderAngle; */

  private static double m_setpoint;

  /** Creates a new Arm. */
  public Shoulder() {

    m_shoulderMotor = new TalonFX(Constants.Shoulder.kLeftArmMotorId);
    m_shoulderMotorFollower = new TalonFX(Constants.Shoulder.kRightArmMotorId);

    /* m_shoulderAngle = new TalonSRX(16);

    m_shoulderAngle.configFactoryDefault();

    m_shoulderAngle.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); */

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

    m_shoulderMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(Constants.Shoulder.kForwardSoftLimit, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configReverseSoftLimitThreshold(Constants.Shoulder.kReverseSoftLimit, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configForwardSoftLimitEnable(true);
    m_shoulderMotor.configReverseSoftLimitEnable(true);

    m_shoulderMotor.selectProfileSlot(0, 0);
    m_shoulderMotor.config_kF(0, Constants.Shoulder.kF);
    m_shoulderMotor.config_kP(0, Constants.Shoulder.kP);
    m_shoulderMotor.config_kI(0, Constants.Shoulder.kI);
    m_shoulderMotor.config_kD(0, Constants.Shoulder.kD);

    m_shoulderMotor.configMotionCruiseVelocity(Constants.Shoulder.kMotionCruiseVelocity, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configMotionAcceleration(Constants.Shoulder.kMotionAcceleration, Constants.TalonFX.kTimeoutMs);
    m_shoulderMotor.configMotionSCurveStrength(Constants.Shoulder.kMotionSCurveStrength);

    m_shoulderMotor.configNeutralDeadband(Constants.Shoulder.kDeadband);
    m_shoulderMotorFollower.configNeutralDeadband(Constants.Shoulder.kDeadband);

    m_shoulderMotor.configVoltageCompSaturation(12.0);
    m_shoulderMotorFollower.configVoltageCompSaturation(12.0);

    m_shoulderMotor.enableVoltageCompensation(true);
    m_shoulderMotorFollower.enableVoltageCompensation(true);

    m_shoulderMotorFollower.follow(m_shoulderMotor);
    m_shoulderMotorFollower.setInverted(TalonFXInvertType.OpposeMaster);

    m_shoulderMotor.configOpenloopRamp(Constants.Shoulder.kRampRate);

    m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
    m_shoulderMotorFollower.setNeutralMode(NeutralMode.Brake);

    m_setpoint = getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /* System.out.println(m_shoulderAngle.getSelectedSensorPosition()); */
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shoulder");
    builder.addDoubleProperty("Shoulder Position", this::getPosition, null);
    builder.addDoubleProperty("Shoulder Angle", this::getAngle, null);
    builder.addDoubleProperty("Shoulder Setpoint", this::getSetpoint, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_shoulderMotor.set(ControlMode.PercentOutput, percentOutput);
    m_setpoint = getPosition();
  }

  public void setPosition(double degrees) {
    setSetpoint(degrees);
    m_shoulderMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public void setSetpoint(double degrees) {
    double ticks = Units.degreesToTicks(degrees, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution);
    m_setpoint = ticks;
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

  public double getAngle() {
    return Units.ticksToDegrees(getPosition(), Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution);
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public boolean atSetpoint(double tolerance) {
    double setpointAngle = Units.ticksToDegrees(m_setpoint, Constants.Shoulder.kMotorToArm, Constants.TalonFX.kEncoderResolution);

    return Math.abs(setpointAngle - getAngle()) < tolerance;
  }
}

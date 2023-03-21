// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Units;

public class Wrist extends SubsystemBase {

  private static TalonFX m_wristMotor;

  private static CANCoder m_encoder;

  private static double m_setpoint;

  /** Creates a new Wrist. */
  public Wrist() {
    m_wristMotor = new TalonFX(Constants.Wrist.kWristMotorId);

    // Encoder Configuration
    m_encoder = new CANCoder(Constants.Wrist.kWristEncoderId);

    m_encoder.configFactoryDefault();
    m_encoder.clearStickyFaults();
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_encoder.configMagnetOffset(Constants.Wrist.kZeroOffset);
    m_encoder.configSensorDirection(false);
    m_encoder.setPositionToAbsolute();

    // Motor Configuration
    m_wristMotor.configFactoryDefault();
    m_wristMotor.clearStickyFaults();

    m_wristMotor.configRemoteFeedbackFilter(m_encoder.getDeviceID(), RemoteSensorSource.CANCoder, 0);
    m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.selectProfileSlot(0, 0);
    m_wristMotor.config_kF(0, Constants.Wrist.kF);
    m_wristMotor.config_kP(0, Constants.Wrist.kP);
    m_wristMotor.config_kI(0, Constants.Wrist.kI);
    m_wristMotor.config_kD(0,Constants.Wrist.kD);

    m_wristMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(Constants.Wrist.kForwardSoftLimit, 1.0, Constants.CANCoder.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configReverseSoftLimitThreshold(Constants.Wrist.kReverseSoftLimit, Constants.TalonFX.kTimeoutMs);
    
    m_wristMotor.configForwardSoftLimitEnable(true);
    m_wristMotor.configReverseSoftLimitEnable(true);

    m_wristMotor.configMotionCruiseVelocity(Constants.Wrist.kMotionCruiseVelocity, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configMotionAcceleration(Constants.Wrist.kMotionAcceleration, Constants.TalonFX.kTimeoutMs); 
    m_wristMotor.configMotionSCurveStrength(Constants.Wrist.kMotionSCurveStrength);

    m_wristMotor.configVoltageCompSaturation(12.0);
    m_wristMotor.enableVoltageCompensation(true);

    m_wristMotor.configNeutralDeadband(Constants.Wrist.kDeadband);

    m_wristMotor.setNeutralMode(NeutralMode.Coast);

    m_wristMotor.setInverted(false);

    m_setpoint = getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Wrist");
    builder.addDoubleProperty("Wrist Position", this::getPosition, null);
    builder.addDoubleProperty("Wrist Angle", this::getAngle, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_wristMotor.set(ControlMode.PercentOutput, percentOutput);
    m_setpoint = getPosition();
  }

  public void setPosition(double degrees) {
    setSetpoint(degrees);
    m_wristMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public void setLastPosition() {
    m_wristMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public void setSetpoint(double degrees) {
    double ticks = Units.degreesToTicks(degrees, 1.0, Constants.CANCoder.kEncoderResolution);
    m_setpoint = ticks;
  }

  public void stop() {
    setPercentOutput(0);
  }

  public double getPosition() {
    return m_wristMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    return Units.ticksToDegrees(getPosition(), 1.0, Constants.CANCoder.kEncoderResolution);
  }

  public void enableLimits() {
    m_wristMotor.configForwardSoftLimitEnable(true, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configReverseSoftLimitEnable(true, Constants.TalonFX.kTimeoutMs);
  }

  public void disableLimits() {
    m_wristMotor.configForwardSoftLimitEnable(false, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configReverseSoftLimitEnable(false, Constants.TalonFX.kTimeoutMs);
  }

  public void zero() {
    m_wristMotor.setSelectedSensorPosition(0.0);
  }

  public boolean atSetpoint(double tolerance) {
    double setpointAngle = Units.ticksToDegrees(m_setpoint, 1.0, Constants.CANCoder.kEncoderResolution);

    return Math.abs(setpointAngle - getAngle()) < tolerance;
  }
}

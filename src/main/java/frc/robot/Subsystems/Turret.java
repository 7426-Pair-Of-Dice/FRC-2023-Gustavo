// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Units;

public class Turret extends SubsystemBase {

  private static TalonFX m_turretMotor;

  private static double m_setpoint;

  /** Creates a new Turret. */
  public Turret() {
    m_turretMotor = new TalonFX(Constants.Turret.kTurretMotorId);

    m_turretMotor.configFactoryDefault();

    m_turretMotor.setInverted(true);

    m_turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_turretMotor.setSelectedSensorPosition(0);

    m_turretMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_turretMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);

    m_turretMotor.selectProfileSlot(0, 0);
    m_turretMotor.config_kF(0, Constants.Turret.kF);
    m_turretMotor.config_kP(0, Constants.Turret.kP);
    m_turretMotor.config_kI(0, Constants.Turret.kI);
    m_turretMotor.config_kD(0, Constants.Turret.kD);

    m_turretMotor.configMotionCruiseVelocity(Constants.Turret.kMotionCruiseVelocity, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configMotionAcceleration(Constants.Turret.kMotionAcceleration, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configMotionSCurveStrength(Constants.Turret.kMotionSCurveStrength);

    m_turretMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(Constants.Turret.kForwardSoftLimit, Constants.Turret.kMotorToTurret, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configReverseSoftLimitThreshold(Units.degreesToTicks(Constants.Turret.kReverseSoftLimit, Constants.Turret.kMotorToTurret, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);

    m_turretMotor.configForwardSoftLimitEnable(true);
    m_turretMotor.configReverseSoftLimitEnable(true);

    m_turretMotor.configNeutralDeadband(Constants.Turret.kDeadband);

    m_turretMotor.setNeutralMode(NeutralMode.Brake);

    m_setpoint = getPosition();
  }

  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Turret");
    builder.addDoubleProperty("Turret Position", this::getPosition, null);
    builder.addDoubleProperty("Turret Angle", this::getAngle, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_turretMotor.set(ControlMode.PercentOutput, percentOutput);
    m_setpoint = getPosition();
  }

  public void setPosition(double degrees) {
    setSetpoint(degrees);
    m_turretMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public void setSetpoint(double degrees) {
    double ticks = Units.degreesToTicks(degrees, Constants.Turret.kMotorToTurret, Constants.TalonFX.kEncoderResolution);
    m_setpoint = ticks;
  }

  public void setLastPosition() {
    m_turretMotor.set(ControlMode.MotionMagic, m_setpoint);
  }

  public void stop() {
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getPosition() {
    return m_turretMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    return Units.ticksToDegrees(getPosition(), Constants.Turret.kMotorToTurret, Constants.TalonFX.kEncoderResolution);
  }

  public boolean atSetpoint(double tolerance) {
    double setpointAngle = Units.ticksToDegrees(m_setpoint, Constants.Turret.kMotorToTurret, Constants.TalonFX.kEncoderResolution);

    return Math.abs(setpointAngle - getAngle()) < tolerance;
  }
}

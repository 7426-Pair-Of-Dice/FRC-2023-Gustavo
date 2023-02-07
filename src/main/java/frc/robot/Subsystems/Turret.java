// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private static TalonFX m_turretMotor;

  private static double m_zeroPosition;

  /** Creates a new Turret. */
  public Turret() {
    m_turretMotor = new TalonFX(Constants.Turret.kTurretMotorId);

    m_turretMotor.configFactoryDefault();

    m_turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    m_zeroPosition = getPosition();

    m_turretMotor.configIntegratedSensorOffset(0);

    m_turretMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_turretMotor.configAllowableClosedloopError(Constants.Turret.kPIDLoopId, 0, Constants.TalonFX.kTimeoutMs);

    m_turretMotor.config_kP(Constants.Turret.kPIDLoopId, Constants.Turret.kP);
    m_turretMotor.config_kI(Constants.Turret.kPIDLoopId, Constants.Turret.kI);
    m_turretMotor.config_kD(Constants.Turret.kPIDLoopId, Constants.Turret.kD);

    m_turretMotor.configForwardSoftLimitThreshold(m_zeroPosition + Constants.Turret.kForwardSoftLimitOffset, Constants.TalonFX.kTimeoutMs);
    m_turretMotor.configReverseSoftLimitThreshold(m_zeroPosition - Constants.Turret.kReverseSoftLimitOffset, Constants.TalonFX.kTimeoutMs);

    m_turretMotor.configForwardSoftLimitEnable(true);
    m_turretMotor.configReverseSoftLimitEnable(true);

  }

  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Turret");
    builder.addDoubleProperty("Turret Position", this::getPosition, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_turretMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    m_turretMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getPosition() {
    return m_turretMotor.getSelectedSensorPosition();
  }
}

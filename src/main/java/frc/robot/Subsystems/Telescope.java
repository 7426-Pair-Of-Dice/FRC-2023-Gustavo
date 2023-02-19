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

public class Telescope extends SubsystemBase {

  private static TalonFX m_telescopeMotor;

  /** Creates a new Telescope. */
  public Telescope() {
    m_telescopeMotor = new TalonFX(Constants.Arm.kTelescopeMotorId);

    m_telescopeMotor.configFactoryDefault();

    m_telescopeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_telescopeMotor.setSelectedSensorPosition(0);

    m_telescopeMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_telescopeMotor.configReverseSoftLimitThreshold(0, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configForwardSoftLimitThreshold(Units.metersToTicks(Units.inchesToMeters(20), Constants.Arm.kMotorToTelescope, Constants.TalonFX.kEncoderResolution, Constants.Arm.kMetersPerRev), Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configReverseSoftLimitEnable(true);
    m_telescopeMotor.configForwardSoftLimitEnable(true);

    m_telescopeMotor.configNeutralDeadband(0.05);

    m_telescopeMotor.setNeutralMode(NeutralMode.Brake);

    m_telescopeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Telescope");
    builder.addDoubleProperty("Telescope Position Ticks", this::getPosition, null);
    builder.addDoubleProperty("Telescope Position Meters", this::getPositionMeters, null);
  }

  public void setPercentOutput(double percentOutput) {
    m_telescopeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    setPercentOutput(0);
  }

  public double getPosition() {
    return m_telescopeMotor.getSelectedSensorPosition();
  }

  public double getPositionMeters() {
    return Units.ticksToMeters(getPosition(), 1, Constants.TalonFX.kEncoderResolution, Constants.Arm.kMetersPerRev);
  }
}

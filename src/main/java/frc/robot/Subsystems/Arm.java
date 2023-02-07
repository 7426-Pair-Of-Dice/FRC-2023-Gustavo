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

public class Arm extends SubsystemBase {

  private static TalonFX m_angleMotor;
  private static TalonFX m_angleMotorFollower;
  private static TalonFX m_telescopeMotor;

  private static double m_telescopeZeroPosition;

  /** Creates a new Arm. */
  public Arm() {

    m_angleMotor = new TalonFX(Constants.Arm.kLeftAngleMotorId);
    m_angleMotorFollower = new TalonFX(Constants.Arm.kRightAngleMotorId);

    m_telescopeMotor = new TalonFX(Constants.Arm.kTelescopeMotorId);

    // Angle Configuration
    m_angleMotor.configFactoryDefault();
    m_angleMotorFollower.configFactoryDefault();

    m_angleMotorFollower.follow(m_angleMotor);
    m_angleMotorFollower.setInverted(TalonFXInvertType.OpposeMaster);

    m_angleMotor.setNeutralMode(NeutralMode.Brake);
    m_angleMotorFollower.setNeutralMode(NeutralMode.Brake);

    // Telescope configuration
    m_telescopeMotor.configFactoryDefault();

    m_telescopeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_telescopeZeroPosition = m_telescopeMotor.getSelectedSensorPosition();

    m_telescopeMotor.configIntegratedSensorOffset(0);

    m_telescopeMotor.configReverseSoftLimitThreshold(m_telescopeZeroPosition);
    m_telescopeMotor.configForwardSoftLimitThreshold(m_telescopeZeroPosition + Constants.Arm.kForwardLimitOffset);
    
    m_telescopeMotor.configReverseSoftLimitEnable(true);
    m_telescopeMotor.configForwardSoftLimitEnable(true);

    m_telescopeMotor.setNeutralMode(NeutralMode.Brake);

    m_telescopeMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Arm");
      builder.addDoubleProperty("Telescope Position", this::getTelescopePosition, null);
  }

  public void setAnglePercentOutput(double percentOutput) {
    m_angleMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setTelescopePercentOutput(double percentOutput) {
    m_telescopeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    m_angleMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getTelescopePosition() {
    return m_telescopeMotor.getSelectedSensorPosition();
  }
}

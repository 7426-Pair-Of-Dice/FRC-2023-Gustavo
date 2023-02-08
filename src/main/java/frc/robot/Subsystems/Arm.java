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
import frc.robot.Utility.Units;

public class Arm extends SubsystemBase {

  private static TalonFX m_armMotor;
  private static TalonFX m_armMotorFollower;
  private static TalonFX m_telescopeMotor;

  private static double m_telescopeZeroPosition;
  private static double m_armZeroPosition;

  /** Creates a new Arm. */
  public Arm() {

    m_armMotor = new TalonFX(Constants.Arm.kLeftArmMotorId);
    m_armMotorFollower = new TalonFX(Constants.Arm.kRightArmMotorId);
    m_telescopeMotor = new TalonFX(Constants.Arm.kTelescopeMotorId);

    // Arm Configuration
    m_armMotor.configFactoryDefault();
    m_armMotorFollower.configFactoryDefault();

    m_armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_armZeroPosition = getArmPosition();

    m_armMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_armMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_armMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);
    
    m_armMotorFollower.follow(m_armMotor);
    m_armMotorFollower.setInverted(TalonFXInvertType.OpposeMaster);

    m_armMotor.setNeutralMode(NeutralMode.Brake);
    m_armMotorFollower.setNeutralMode(NeutralMode.Brake);

    // Telescope configuration
    m_telescopeMotor.configFactoryDefault();

    m_telescopeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TalonFX.kTimeoutMs);

    m_telescopeZeroPosition = getTelescopePosition();

    m_telescopeMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_telescopeMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_telescopeMotor.configIntegratedSensorOffset(0);

    m_telescopeMotor.configReverseSoftLimitThreshold(m_telescopeZeroPosition);
    m_telescopeMotor.configForwardSoftLimitThreshold(m_telescopeZeroPosition + Units.metersToTicks(Units.inchesToMeters(20), 1, Constants.TalonFX.kEncoderResolution, Constants.Arm.kMetersPerRev));
    
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
      builder.addDoubleProperty("Telescope Position Ticks", this::getTelescopePosition, null);
      builder.addDoubleProperty("Telescope Position Meters", this::getTelescopePositionMeters, null);
      builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
  }

  public void setArmPercentOutput(double percentOutput) {
    m_armMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setTelescopePercentOutput(double percentOutput) {
    m_telescopeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void stop() {
    m_armMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getArmPosition() {
    return m_armMotor.getSelectedSensorPosition();
  }

  public double getTelescopePosition() {
    return m_telescopeMotor.getSelectedSensorPosition();
  }

  public double getTelescopeOffset() {
    return getTelescopePosition() - m_telescopeZeroPosition;
  }

  public double getTelescopePositionMeters() {
    return Units.ticksToMeters(getTelescopeOffset(), 1, Constants.TalonFX.kEncoderResolution, Constants.Arm.kMetersPerRev);
  }
}

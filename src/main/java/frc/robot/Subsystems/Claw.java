// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private static TalonFX m_wristMotor;

  private static VictorSPX m_backIntakeMotor;
  private static VictorSPX m_frontIntakeMotor;

  /** Creates a new Claw. */
  public Claw() {
    m_wristMotor = new TalonFX(Constants.Claw.kWristMotorId);

    m_backIntakeMotor = new VictorSPX(Constants.Claw.kBackIntakeMotorId);
    m_frontIntakeMotor = new VictorSPX(Constants.Claw.kFrontIntakeMotorId);

    // Wrist configuration
    m_wristMotor.configFactoryDefault();

    m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    m_wristMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.configNeutralDeadband(0.05);

    m_wristMotor.setNeutralMode(NeutralMode.Brake);

    // Intake Configuration
    m_backIntakeMotor.configNeutralDeadband(0.05);
    m_frontIntakeMotor.configNeutralDeadband(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWristPercentOutput(double percentOutput) {
    m_wristMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setIntakePercentOutput(double backPercentOutput, double frontPercentOutput) {
    m_frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, frontPercentOutput);
    m_backIntakeMotor.set(VictorSPXControlMode.PercentOutput, backPercentOutput);
  }

  public void stopWrist() {
    setWristPercentOutput(0);
  }

  public void stopIntake() {
    setIntakePercentOutput(0, 0);
  }
}

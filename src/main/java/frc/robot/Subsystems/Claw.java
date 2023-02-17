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
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Units;

public class Claw extends SubsystemBase {

  private static TalonFX m_wristMotor;

  private static VictorSPX m_backIntakeMotor;
  private static VictorSPX m_frontIntakeMotor;

  private static Rev2mDistanceSensor m_coneDistanceSensor;
  
  private static Ultrasonic m_cubeDistanceSensor;

  private static Pigeon2 m_gyro;

  private static double m_lastWristPosition;

  /** Creates a new Claw. */
  public Claw() {
    m_wristMotor = new TalonFX(Constants.Claw.kWristMotorId);

    m_backIntakeMotor = new VictorSPX(Constants.Claw.kBackIntakeMotorId);
    m_frontIntakeMotor = new VictorSPX(Constants.Claw.kFrontIntakeMotorId);

    m_coneDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kDefault);

    m_cubeDistanceSensor = new Ultrasonic(Constants.Sensors.kClawSonarPingChannel, Constants.Sensors.kClawSonarEchoChannel); 

    m_gyro = new Pigeon2(Constants.Sensors.kClawGyroId);

    m_coneDistanceSensor.setAutomaticMode(true);
    
    Ultrasonic.setAutomaticMode(true);

    // Wrist configuration
    m_wristMotor.configFactoryDefault();

    m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    m_wristMotor.setSelectedSensorPosition(0);

    m_wristMotor.configNominalOutputForward(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configNominalOutputReverse(0, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputForward(1, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configPeakOutputReverse(-1, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.selectProfileSlot(0, 0);
    m_wristMotor.config_kF(0, 0.2);
    m_wristMotor.config_kP(0, 0.08);
    m_wristMotor.config_kI(0, 0);
    m_wristMotor.config_kD(0,0);

    m_wristMotor.configAllowableClosedloopError(0, 0, Constants.TalonFX.kTimeoutMs);

    m_wristMotor.configForwardSoftLimitThreshold(Units.degreesToTicks(120, Constants.Claw.kMotorToWrist, Constants.TalonFX.kEncoderResolution), Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configReverseSoftLimitThreshold(0, Constants.TalonFX.kTimeoutMs);
    
    m_wristMotor.configForwardSoftLimitEnable(true);
    m_wristMotor.configReverseSoftLimitEnable(true);

    m_wristMotor.configMotionCruiseVelocity(10000, Constants.TalonFX.kTimeoutMs);
    m_wristMotor.configMotionAcceleration(8000, Constants.TalonFX.kTimeoutMs); 

    m_wristMotor.configNeutralDeadband(0.05);

    m_wristMotor.setNeutralMode(NeutralMode.Brake);

    m_wristMotor.setInverted(true);

    m_lastWristPosition = getPosition();

    // Intake Configuration
    m_backIntakeMotor.configNeutralDeadband(0.05);
    m_frontIntakeMotor.configNeutralDeadband(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Claw");
    builder.addDoubleProperty("Claw Cone Distance", this::getConeRange, null);
    builder.addDoubleProperty("Claw Cube Distance", this::getCubeRange, null);
    builder.addDoubleProperty("Claw Yaw", this::getYaw, null);
    builder.addDoubleProperty("Claw Pitch", this::getPitch, null);
    builder.addDoubleProperty("Claw Roll", this::getRoll, null);
    builder.addDoubleProperty("Claw Position", this::getPosition, null);
    builder.addDoubleProperty("Claw Angle", this::getAngle, null);
  }

  public void setWristPercentOutput(double percentOutput) {
    m_wristMotor.set(ControlMode.PercentOutput, percentOutput);
    m_lastWristPosition = getPosition();
  }

  public void setIntakePercentOutput(double backPercentOutput, double frontPercentOutput) {
    m_frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, frontPercentOutput);
    m_backIntakeMotor.set(VictorSPXControlMode.PercentOutput, backPercentOutput);
  }

  public void setWristPosition(double degrees) {
    double ticks = Units.degreesToTicks(degrees, Constants.Claw.kMotorToWrist, Constants.TalonFX.kEncoderResolution);
    m_lastWristPosition = ticks;
    m_wristMotor.set(ControlMode.MotionMagic, ticks);
  }

  public void setWristLastPosition() {
    m_wristMotor.set(ControlMode.MotionMagic, m_lastWristPosition);
  }

  public void stopWrist() {
    setWristPercentOutput(0);
  }

  public void stopIntake() {
    setIntakePercentOutput(0, 0);
  }

  public void zero() {
    m_wristMotor.setSelectedSensorPosition(0);
  }

  public double getConeRange() { return m_coneDistanceSensor.GetRange(); }

  public double getCubeRange() { return m_cubeDistanceSensor.getRangeInches(); }

  public double getYaw() { return m_gyro.getYaw(); }

  public double getPitch() { return m_gyro.getPitch(); }

  public double getRoll() { return m_gyro.getRoll(); }

  public double getPosition() { return m_wristMotor.getSelectedSensorPosition(); }

  public double getAngle() { return Units.ticksToDegrees(getPosition(), Constants.Claw.kMotorToWrist, Constants.TalonFX.kEncoderResolution); }
}

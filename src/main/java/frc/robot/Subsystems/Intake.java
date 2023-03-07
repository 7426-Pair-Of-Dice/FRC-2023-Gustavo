// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static VictorSPX m_leftIntakeMotor;
  private static VictorSPX m_rightIntakeMotor;

  private static Rev2mDistanceSensor m_coneDistanceSensor;
  private static Ultrasonic m_cubeDistanceSensor;

  private static double m_coneRange;
  private static double m_cubeRange;

  /** Creates a new Intake. */
  public Intake() {
    m_leftIntakeMotor = new VictorSPX(Constants.Intake.kLeftIntakeMotorId);
    m_rightIntakeMotor = new VictorSPX(Constants.Intake.kRightIntakeMotorId);

    m_coneDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kDefault);
    m_cubeDistanceSensor = new Ultrasonic(Constants.Sensors.kClawSonarPingChannel, Constants.Sensors.kClawSonarEchoChannel);
    
    m_coneDistanceSensor.setAutomaticMode(true);
    
    Ultrasonic.setAutomaticMode(true);

    m_leftIntakeMotor.setInverted(true);

    m_leftIntakeMotor.configNeutralDeadband(0.05);
    m_rightIntakeMotor.configNeutralDeadband(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_coneRange = m_coneDistanceSensor.getRange();
    m_cubeRange = m_cubeDistanceSensor.getRangeInches();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Cone Range", this::getConeRange, null);
    builder.addDoubleProperty("Cube Range", this::getCubeRange, null);
  }

  private void setPercentOutput(double leftPercentOutput, double rightPercentOutput) {
    m_leftIntakeMotor.set(VictorSPXControlMode.PercentOutput, leftPercentOutput);
    m_rightIntakeMotor.set(VictorSPXControlMode.PercentOutput, rightPercentOutput);
  }

  public void intakeCone() {
    setPercentOutput(-1, 1);
  }

  public void intakeCube() {
    setPercentOutput(1, -1);
  }

  public void releaseCone() {
    setPercentOutput(1, -1);
  }

  public void releaseCube() {
    setPercentOutput(-1, 1);
  }

  public void stop() {
    setPercentOutput(0, 0);
  }

  public double getConeRange() { 
    return m_coneRange; 
  }

  public double getCubeRange() { 
    return m_cubeRange; 
  }

  public boolean getConeDetected() {
    return false;
  }

  public boolean getCubeDetected() {
    return m_cubeRange < 10.0;
  }
}

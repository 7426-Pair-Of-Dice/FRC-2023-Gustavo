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

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static VictorSPX m_backIntakeMotor;
  private static VictorSPX m_frontIntakeMotor;

  private static Rev2mDistanceSensor m_coneDistanceSensor;
  
  private static Ultrasonic m_cubeDistanceSensor;

  /** Creates a new Intake. */
  public Intake() {
    m_backIntakeMotor = new VictorSPX(Constants.Claw.kBackIntakeMotorId);
    m_frontIntakeMotor = new VictorSPX(Constants.Claw.kFrontIntakeMotorId);

    m_coneDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kDefault);
    m_cubeDistanceSensor = new Ultrasonic(Constants.Sensors.kClawSonarPingChannel, Constants.Sensors.kClawSonarEchoChannel);
    
    m_coneDistanceSensor.setAutomaticMode(true);
    
    Ultrasonic.setAutomaticMode(true);

    m_backIntakeMotor.configNeutralDeadband(0.05);
    m_frontIntakeMotor.configNeutralDeadband(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentOutput(double backPercentOutput, double frontPercentOutput) {
    m_frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, frontPercentOutput);
    m_backIntakeMotor.set(VictorSPXControlMode.PercentOutput, backPercentOutput);
  }

  public void stopIntake() {
    setPercentOutput(0, 0);
  }

  public double getConeRange() { 
    return m_coneDistanceSensor.GetRange(); 
  }

  public double getCubeRange() { 
    return m_cubeDistanceSensor.getRangeInches(); 
  }
}

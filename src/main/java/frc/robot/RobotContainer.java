// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {

  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Arm m_arm;
  private static Claw m_claw;

  private static DriveTeleop m_driveTeleop;
  private static RunCommand m_turretRight;
  private static RunCommand m_turretLeft;
  private static InstantCommand m_stopTurret;
  private static ArmTeleop m_armTeleop;

  private static RunCommand m_intakeCone;
  private static RunCommand m_intakeCube;
  private static RunCommand m_releaseCone;
  private static RunCommand m_releaseCube;
  private static InstantCommand m_stopIntake;

  private static CommandXboxController m_driverController;
  private static CommandXboxController m_operatorController;

  public RobotContainer() {
    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorController = new CommandXboxController(Constants.Input.kOperatorControllerId);

    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_arm = new Arm();
    m_claw = new Claw();

    // Commands
    m_driveTeleop = new DriveTeleop(m_driveTrain, m_driverController);
    m_armTeleop = new ArmTeleop(m_arm, m_operatorController);

    m_turretRight = new RunCommand(() -> m_turret.setPercentOutput(-0.25), m_turret);
    m_turretLeft = new RunCommand(() -> m_turret.setPercentOutput(0.25), m_turret);
    m_stopTurret = new InstantCommand(() -> m_turret.stop(), m_turret);

    m_intakeCone = new RunCommand(() -> m_claw.setIntakePercentOutput(Constants.Claw.kIntakePercentOutput, Constants.Claw.kIntakePercentOutput), m_claw);
    m_releaseCone = new RunCommand(() -> m_claw.setIntakePercentOutput(-Constants.Claw.kIntakePercentOutput, -Constants.Claw.kIntakePercentOutput), m_claw);
    m_intakeCube = new RunCommand(() -> m_claw.setIntakePercentOutput(-Constants.Claw.kIntakePercentOutput, -Constants.Claw.kIntakePercentOutput), m_claw);
    m_releaseCube = new RunCommand(() -> m_claw.setIntakePercentOutput(Constants.Claw.kIntakePercentOutput, Constants.Claw.kIntakePercentOutput), m_claw);
    m_stopIntake = new InstantCommand(() -> m_claw.stopIntake(), m_claw);

    m_driveTrain.setDefaultCommand(m_driveTeleop);
    m_arm.setDefaultCommand(m_armTeleop);

    // Configures controller and joystick bindings
    configureBindings();
  }
  

  private void configureBindings() {
    m_operatorController.leftTrigger().whileTrue(m_turretLeft).onFalse(m_stopTurret);
    m_operatorController.rightTrigger().whileTrue(m_turretRight).onFalse(m_stopTurret);

    m_operatorController.y().whileTrue(m_intakeCone).onFalse(m_stopIntake);
    m_operatorController.a().whileTrue(m_releaseCone).onFalse(m_stopIntake);

    m_operatorController.x().whileTrue(m_intakeCube).onFalse(m_stopIntake);
    m_operatorController.b().whileTrue(m_releaseCube).onFalse(m_stopIntake);
  }

  public void updateDashboard() {
    SmartDashboard.putData(m_driveTrain);
    SmartDashboard.putData(m_turret);
    SmartDashboard.putData(m_arm);
    SmartDashboard.putData(m_claw);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

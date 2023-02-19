// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {

  // Controllers
  private static CommandXboxController m_driverController;
  private static CommandJoystick m_operatorJoystickLeft;
  private static CommandJoystick m_operatorJoystickRight;

  // Subsystems
  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Arm m_arm;
  private static Wrist m_wrist;
  private static Intake m_intake;
  private static Telescope m_telescope;

  // Commands
  private static RunCommand m_tankDrive;
  private static InstantCommand m_stopDrive;

  private static RunCommand m_armMaintain;
  private static RunCommand m_armUp;
  private static RunCommand m_armDown;

  private static RunCommand m_telescopeControl;
  private static InstantCommand m_stopTelescope;

  private static RunCommand m_turretControl;
  private static InstantCommand m_stopTurret;

  private static RunCommand m_wristMaintain;
  private static RunCommand m_wristForward;
  private static RunCommand m_wristBackward;

  private static WrapperCommand m_intakeCone;
  private static WrapperCommand m_releaseCone;
  private static WrapperCommand m_intakeCube;
  private static WrapperCommand m_releaseCube;
  private static InstantCommand m_stopIntake;

  public RobotContainer() {
    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorJoystickLeft = new CommandJoystick(Constants.Input.kOperatorJoystickLeftId);
    m_operatorJoystickRight = new CommandJoystick(Constants.Input.kOperatorJoystickRightId);

    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_arm = new Arm();
    m_wrist = new Wrist();
    m_intake = new Intake();
    m_telescope = new Telescope();

    // Commands
    m_tankDrive = new RunCommand(() -> m_driveTrain.tankDrive(-m_driverController.getLeftY() * 0.7, -m_driverController.getLeftY() * 0.7), m_driveTrain);
    m_stopDrive = new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain);

    m_armMaintain = new RunCommand(() -> m_arm.setLastPosition(), m_arm);
    m_armUp = new RunCommand(() -> m_arm.setPercentOutput(0.3), m_arm);
    m_armDown = new RunCommand(() -> m_arm.setPercentOutput(-0.3), m_arm);

    m_telescopeControl = new RunCommand(() -> m_telescope.setPercentOutput(m_operatorJoystickLeft.getY() * 0.25), m_telescope);
    m_stopTelescope = new InstantCommand(() -> m_telescope.stop(), m_telescope);

    m_turretControl = new RunCommand(() -> m_turret.setPercentOutput(m_operatorJoystickRight.getZ() * 0.25), m_turret);
    m_stopTurret = new InstantCommand(() -> m_turret.stop(), m_turret);

    m_wristMaintain = new RunCommand(() -> m_wrist.setLastPosition(), m_wrist);
    m_wristForward = new RunCommand(() -> m_wrist.setPercentOutput(0.3), m_wrist);
    m_wristBackward = new RunCommand(() -> m_wrist.setPercentOutput(-0.3), m_wrist);

    m_intakeCone = new RunCommand(() -> m_intake.setPercentOutput(Constants.Claw.kIntakePercentOutput, Constants.Claw.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_releaseCone = new RunCommand(() -> m_intake.setPercentOutput(-Constants.Claw.kIntakePercentOutput, -Constants.Claw.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_intakeCube = new RunCommand(() -> m_intake.setPercentOutput(-Constants.Claw.kIntakePercentOutput, -Constants.Claw.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_releaseCube = new RunCommand(() -> m_intake.setPercentOutput(Constants.Claw.kIntakePercentOutput, Constants.Claw.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_stopIntake = new InstantCommand(() -> m_intake.stopIntake(), m_intake);

    m_arm.setDefaultCommand(m_armMaintain);
    m_wrist.setDefaultCommand(m_wristMaintain);

    // Configures controller and joystick bindings
    configureBindings();
  }
  

  private void configureBindings() {
    m_driverController.leftStick().or(m_driverController.rightStick()).whileTrue(m_tankDrive).onFalse(m_stopDrive);

    m_operatorJoystickRight.povUp().whileTrue(m_armUp);
    m_operatorJoystickRight.povDown().whileTrue(m_armDown);
    m_operatorJoystickRight.axisGreaterThan(AxisType.kZ.value, 0.05).or(m_operatorJoystickRight.axisLessThan(AxisType.kZ.value, -0.05)).whileTrue(m_turretControl).onFalse(m_stopTurret);
    m_operatorJoystickRight.button(Constants.Input.kIntakeConeButton).whileTrue(m_intakeCone).onFalse(m_stopIntake);
    m_operatorJoystickRight.button(Constants.Input.kReleaseConeButton).whileTrue(m_releaseCone).onFalse(m_stopIntake);

    m_operatorJoystickLeft.povUp().whileTrue(m_wristForward);
    m_operatorJoystickLeft.povDown().whileTrue(m_wristBackward);
    m_operatorJoystickLeft.axisGreaterThan(AxisType.kY.value, 0.05).or(m_operatorJoystickLeft.axisLessThan(AxisType.kY.value, -0.05)).whileTrue(m_telescopeControl).onFalse(m_stopTelescope);
    m_operatorJoystickLeft.button(Constants.Input.kIntakeCubeButton).whileTrue(m_intakeCube).onFalse(m_stopIntake);
    m_operatorJoystickLeft.button(Constants.Input.kReleaseCubeButton).whileTrue(m_releaseCube).onFalse(m_stopIntake);
  }

  public void updateDashboard() {
    SmartDashboard.putData(m_driveTrain);
    SmartDashboard.putData(m_turret);
    SmartDashboard.putData(m_arm);
    SmartDashboard.putData(m_wrist);
    SmartDashboard.putData(m_intake);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

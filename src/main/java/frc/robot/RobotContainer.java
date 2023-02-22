// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.ArmPreset;
import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {

  // Input
  private static CommandXboxController m_driverController;
  private static CommandJoystick m_operatorJoystick;

  private static Trigger m_joystickPovUp;
  private static Trigger m_joystickPovDown;
  private static Trigger m_joystickZAxis;
  private static Trigger m_joystickYAxis;
  private static Trigger m_joystickIntakeCubeButton;
  private static Trigger m_joystickIntakeConeFrontButton;
  private static Trigger m_joystickIntakeConeBottomButton;
  private static Trigger m_joystickIntakeReleaseButton;
  private static Trigger m_joystickWristForwardButton;
  private static Trigger m_joystickWristBackwardButton;
  private static Trigger m_joystickHomePresetButton;

  // Subsystems
  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Shoulder m_shoulder;
  private static Wrist m_wrist;
  private static Intake m_intake;
  private static Telescope m_telescope;

  // Commands
  private static RunCommand m_tankDrive;

  private static RunCommand m_shoulderMaintain;
  private static RunCommand m_shoulderUp;
  private static RunCommand m_shoulderDown;

  private static RunCommand m_telescopeControl;
  private static InstantCommand m_stopTelescope;

  private static RunCommand m_turretMaintain;
  private static RunCommand m_turretControl;
  private static InstantCommand m_stopTurret;

  private static RunCommand m_wristMaintain;
  private static RunCommand m_wristForward;
  private static RunCommand m_wristBackward;

  private static WrapperCommand m_intakeCube;
  private static WrapperCommand m_intakeConeFront;
  private static WrapperCommand m_intakeConeBottom;
  private static WrapperCommand m_releaseCube;
  private static WrapperCommand m_releaseConeFront;
  private static WrapperCommand m_releaseConeBottom;
  private static InstantCommand m_stopIntake;

  private static ArmPreset m_homePreset;

  public RobotContainer() {
    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorJoystick = new CommandJoystick(Constants.Input.kOperatorJoystickId);

    m_joystickPovUp = m_operatorJoystick.povUp();
    m_joystickPovDown = m_operatorJoystick.povDown();
    m_joystickZAxis = m_operatorJoystick.axisGreaterThan(AxisType.kZ.value, 0.05).or(m_operatorJoystick.axisLessThan(AxisType.kZ.value, -0.05));
    m_joystickYAxis = m_operatorJoystick.axisGreaterThan(AxisType.kY.value, 0.05).or(m_operatorJoystick.axisLessThan(AxisType.kY.value, -0.05));
    m_joystickIntakeCubeButton =  m_operatorJoystick.button(Constants.Input.kIntakeCubeButtonId);
    m_joystickIntakeConeFrontButton = m_operatorJoystick.button(Constants.Input.kIntakeConeFrontButtonId);
    m_joystickIntakeConeBottomButton = m_operatorJoystick.button(Constants.Input.kIntakeConeBottomButtonId);
    m_joystickWristForwardButton = m_operatorJoystick.button(Constants.Input.kWristForwardButton);
    m_joystickWristBackwardButton = m_operatorJoystick.button(Constants.Input.kWristBackwardButton);
    m_joystickIntakeReleaseButton = m_operatorJoystick.button(Constants.Input.kIntakeReleaseButtonId);
    m_joystickHomePresetButton = m_operatorJoystick.button(Constants.Input.kHomePresetButtonId);

    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_shoulder = new Shoulder();
    m_wrist = new Wrist();
    m_intake = new Intake();
    m_telescope = new Telescope();

    // Commands
    m_tankDrive = new RunCommand(() -> m_driveTrain.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY()), m_driveTrain);

    m_shoulderMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_shoulder);
    m_shoulderUp = new RunCommand(() -> m_shoulder.setPercentOutput(0.5), m_shoulder);
    m_shoulderDown = new RunCommand(() -> m_shoulder.setPercentOutput(-0.25), m_shoulder);

    m_telescopeControl = new RunCommand(() -> m_telescope.setPercentOutput(-m_operatorJoystick.getY() * 0.5), m_telescope);
    m_stopTelescope = new InstantCommand(() -> m_telescope.stop(), m_telescope);

    m_turretMaintain = new RunCommand(() -> m_turret.setLastPosition(), m_turret);
    m_turretControl = new RunCommand(() -> m_turret.setPercentOutput(-m_operatorJoystick.getZ() * 0.25), m_turret);
    m_stopTurret = new InstantCommand(() -> m_turret.stop(), m_turret);

    m_wristMaintain = new RunCommand(() -> m_wrist.setLastPosition(), m_wrist);
    m_wristForward = new RunCommand(() -> m_wrist.setPercentOutput(0.3), m_wrist);
    m_wristBackward = new RunCommand(() -> m_wrist.setPercentOutput(-0.3), m_wrist);

    m_intakeCube = new RunCommand(() -> m_intake.setPercentOutput(Constants.Intake.kIntakePercentOutput, -Constants.Intake.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_intakeConeFront = new RunCommand(() -> m_intake.setPercentOutput(-Constants.Intake.kIntakePercentOutput, -Constants.Intake.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_intakeConeBottom = new RunCommand(() -> m_intake.setPercentOutput(-Constants.Intake.kIntakePercentOutput, Constants.Intake.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_releaseCube = new RunCommand(() -> m_intake.setPercentOutput(-Constants.Intake.kIntakePercentOutput, Constants.Intake.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_releaseConeFront = new RunCommand(() -> m_intake.setPercentOutput(Constants.Intake.kIntakePercentOutput, Constants.Intake.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_releaseConeBottom = new RunCommand(() -> m_intake.setPercentOutput(Constants.Intake.kIntakePercentOutput, -Constants.Intake.kIntakePercentOutput), m_intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_stopIntake = new InstantCommand(() -> m_intake.stopIntake(), m_intake);

    m_homePreset = new ArmPreset(m_shoulder, m_telescope, m_wrist, 0, 0, 0);

    m_driveTrain.setDefaultCommand(m_tankDrive);
    m_turret.setDefaultCommand(m_turretMaintain);
    m_shoulder.setDefaultCommand(m_shoulderMaintain);
    m_wrist.setDefaultCommand(m_wristMaintain);

    // Configures controller and joystick bindings
    configureBindings();
  }
  
  private void configureBindings() {
    m_joystickPovUp.whileTrue(m_shoulderUp);
    m_joystickPovDown.whileTrue(m_shoulderDown);

    m_joystickZAxis.whileTrue(m_turretControl).onFalse(m_stopTurret);
    m_joystickYAxis.whileTrue(m_telescopeControl).onFalse(m_stopTelescope);

    m_joystickIntakeCubeButton.and(m_joystickIntakeReleaseButton.negate()).whileTrue(m_intakeCube).onFalse(m_stopIntake);
    m_joystickIntakeConeFrontButton.and(m_joystickIntakeReleaseButton.negate()).whileTrue(m_intakeConeFront).onFalse(m_stopIntake);
    m_joystickIntakeConeBottomButton.and(m_joystickIntakeReleaseButton.negate()).whileTrue(m_intakeConeBottom).onFalse(m_stopIntake);

    m_joystickIntakeReleaseButton.and(m_joystickIntakeCubeButton).whileTrue(m_releaseCube).onFalse(m_stopIntake);
    m_joystickIntakeReleaseButton.and(m_joystickIntakeConeFrontButton).whileTrue(m_releaseConeFront).onFalse(m_stopIntake);
    m_joystickIntakeReleaseButton.and(m_joystickIntakeConeBottomButton).whileTrue(m_releaseConeBottom).onFalse(m_stopIntake);

    m_joystickWristForwardButton.whileTrue(m_wristBackward);
    m_joystickWristBackwardButton.whileTrue(m_wristForward);

    m_joystickHomePresetButton.onTrue(m_homePreset);
  }

  public void updateDashboard() {
    SmartDashboard.putData(m_driveTrain);
    SmartDashboard.putData(m_turret);
    SmartDashboard.putData(m_shoulder);
    SmartDashboard.putData(m_wrist);
    SmartDashboard.putData(m_intake);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

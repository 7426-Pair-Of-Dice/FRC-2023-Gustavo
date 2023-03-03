// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Commands.*;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {

  // Input
  private static CommandXboxController m_driverController;
  private static CommandJoystick m_operatorJoystick;

  private static Trigger m_driveStraightTrigger;

  private static Trigger m_doublePlayerStationCubePresetTrigger;
  private static Trigger m_singlePlayerStationCubePresetTrigger;
  private static Trigger m_floorCubePresetTrigger;

  private static Trigger m_doublePlayerStationConePresetTrigger;
  private static Trigger m_singlePlayerStationConePresetTrigger;
  private static Trigger m_floorConePresetTrigger;

  private static Trigger m_topScoreCubePresetTrigger;
  private static Trigger m_middleScoreCubePresetTrigger;
  private static Trigger m_bottomScoreCubePresetTrigger;

  private static Trigger m_topScoreConePresetTrigger;
  private static Trigger m_middleScoreConePresetTrigger;
  private static Trigger m_bottomScoreConePresetTrigger;

  private static Trigger m_telescopeControlTrigger;
  private static Trigger m_turretControlTrigger;

  private static Trigger m_shoulderUpTrigger;
  private static Trigger m_shoulderDownTrigger;
  private static Trigger m_wristUpTrigger;
  private static Trigger m_wristDownTrigger;
  private static Trigger m_releaseTrigger;

  private static Trigger m_turretTrackingTrigger;

  // Subsystems
  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Shoulder m_shoulder;
  private static Wrist m_wrist;
  private static Intake m_intake;
  private static Telescope m_telescope;
  private static Limelight m_limelight;

  // Commands
  private static RunCommand m_arcadeDrive;
  private static DriveStraight m_driveStraight;

  private static SequentialCommandGroup m_homePreset;

  private static RunCommand m_shoulderUp;
  private static RunCommand m_shoulderDown;
  private static RunCommand m_wristUp;
  private static RunCommand m_wristDown;
  private static RunCommand m_telescopeControl;
  private static RunCommand m_turretControl;
  private static RunCommand m_releaseCube;
  private static RunCommand m_releaseCone;

  private static PIDCommand m_turretTracking;

  private static InstantCommand m_turretStop;
  private static InstantCommand m_shoulderStop;
  private static InstantCommand m_telescopeStop;
  private static InstantCommand m_wristStop;
  private static InstantCommand m_intakeStop;
  private static InstantCommand m_driveStop;

  private static RunCommand m_shoulderMaintain;
  private static RunCommand m_telescopeMaintain;
  private static RunCommand m_wristMaintain;

  private static WrapperCommand m_doublePlayerStationCubePreset;
  private static WrapperCommand m_singlePlayerStationCubePreset;
  private static WrapperCommand m_floorCubePreset;

  private static WrapperCommand m_doublePlayerStationConePreset;
  private static WrapperCommand m_singlePlayerStationConePreset;
  private static WrapperCommand m_floorConePreset;

  private static WrapperCommand m_topScoreCubePreset;
  private static WrapperCommand m_middleScoreCubePreset;
  private static WrapperCommand m_bottomScoreCubePreset;

  private static WrapperCommand m_topScoreConePreset;
  private static WrapperCommand m_middleScoreConePreset;
  private static WrapperCommand m_bottomScoreConePreset;


  public RobotContainer() {
    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorJoystick = new CommandJoystick(Constants.Input.kOperatorJoystickId);

    m_driveStraightTrigger = m_driverController.rightTrigger();

    m_doublePlayerStationCubePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickRightTopLeftButtonId);
    m_singlePlayerStationCubePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickRightTopMiddleButtonId);
    m_floorCubePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickRightTopRightButtonId);

    m_doublePlayerStationConePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickRightBottomLeftButtonId);
    m_singlePlayerStationConePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickRightBottomMiddleButtonId);
    m_floorConePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickRightBottomRightButtonId);

    m_topScoreCubePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickLeftTopLeftButtonId);
    m_middleScoreCubePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickLeftTopMiddleButtonId);
    m_bottomScoreCubePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickLeftTopRightButtonId);

    m_topScoreConePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickLeftBottomLeftButtonId);
    m_middleScoreConePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickLeftBottomMiddleButtonId);
    m_bottomScoreConePresetTrigger = m_operatorJoystick.button(Constants.Input.kJoystickLeftBottomRightButtonId);

    m_telescopeControlTrigger = m_operatorJoystick.axisGreaterThan(AxisType.kY.value, 0.1).or(m_operatorJoystick.axisLessThan(AxisType.kY.value, -0.1));
    m_turretControlTrigger = m_operatorJoystick.axisGreaterThan(AxisType.kZ.value, 0.1).or(m_operatorJoystick.axisLessThan(AxisType.kZ.value, -0.1));

    m_shoulderUpTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId).negate());
    m_shoulderDownTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId));
    m_wristUpTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId));
    m_wristDownTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId).negate());
    m_releaseTrigger = m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId);

    m_turretTrackingTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterMiddleButtonId);

    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_shoulder = new Shoulder();
    m_wrist = new Wrist();
    m_intake = new Intake();
    m_telescope = new Telescope();
    m_limelight = new Limelight();

    // Commands
    m_arcadeDrive = new RunCommand(
      () -> {
        m_driveTrain.arcadeDrive(-m_driverController.getLeftY() * (1 - (m_shoulder.getAngle() / 100.0)), m_driverController.getRightX() * (1 - (m_shoulder.getAngle() / 100.0 * 2.0)));
      }, 
      m_driveTrain
    );

    m_driveStraight = new DriveStraight(m_driveTrain, m_driverController);

    m_telescopeControl = new RunCommand(() -> m_telescope.setPercentOutput(-m_operatorJoystick.getY() * 0.6), m_telescope);
    m_turretControl = new RunCommand(() -> m_turret.setPercentOutput(-m_operatorJoystick.getZ()), m_turret);
    m_shoulderUp = new RunCommand(() -> m_shoulder.setPercentOutput(0.5), m_shoulder);
    m_shoulderDown = new RunCommand(() -> m_shoulder.setPercentOutput(-0.5), m_shoulder);
    m_wristUp = new RunCommand(() -> m_wrist.setPercentOutput(-0.25), m_wrist);
    m_wristDown = new RunCommand(() -> m_wrist.setPercentOutput(0.25), m_wrist);
    m_releaseCube = new RunCommand(() -> m_intake.releaseCube(), m_intake);
    m_releaseCone = new RunCommand(() -> m_intake.releaseCone(), m_intake);

    m_turretTracking = new PIDCommand(
      new PIDController(0.04, 0.0, 0.0), 
      m_limelight::getXOffset, 
      0, 
      m_turret::setPercentOutput, 
      m_turret, 
      m_limelight
    );

    m_turretStop = new InstantCommand(() -> m_turret.stop(), m_turret);
    m_shoulderStop = new InstantCommand(() -> m_shoulder.stop(), m_shoulder);
    m_telescopeStop = new InstantCommand(() -> m_telescope.stop(), m_telescope);
    m_wristStop = new InstantCommand(() -> m_wrist.stop(), m_wrist);
    m_intakeStop = new InstantCommand(() -> m_intake.stop(), m_intake);
    m_driveStop = new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain);

    m_shoulderMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_shoulder);
    m_telescopeMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_telescope);
    m_wristMaintain = new RunCommand(() -> m_wrist.setLastPosition(), m_wrist);

    m_homePreset = new SequentialCommandGroup(
      new InstantCommand(() -> m_intake.stop(), m_intake),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new ShoulderPreset(m_shoulder, 0)
    );

    // Cube grabbing presets
    m_doublePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake),
      new ShoulderPreset(m_shoulder, 65.0),
      new WristPreset(m_wrist, 124.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_singlePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake),
      new ShoulderPreset(m_shoulder, 30.0),
      new WristPreset(m_wrist, 0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_floorCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake),
      new ShoulderPreset(m_shoulder, 10),
      new WristPreset(m_wrist, 125.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    // Cone grabbing presets
    m_doublePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake),
      new ShoulderPreset(m_shoulder, 63.0),
      new WristPreset(m_wrist, 78.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_singlePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake),
      new ShoulderPreset(m_shoulder, 40.0),
      new WristPreset(m_wrist,22.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_floorConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake),
      new ShoulderPreset(m_shoulder, 0),
      new WristPreset(m_wrist, 40.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    // Cube scoring presets
    m_topScoreCubePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 67.0),
      new WristPreset(m_wrist, 115.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_middleScoreCubePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 50.0),
      new WristPreset(m_wrist, 105.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_bottomScoreCubePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0.0),
      new WristPreset(m_wrist, 55.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    // Cone scoring presets
    m_topScoreConePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 80.0),
      new WristPreset(m_wrist, 80.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_middleScoreConePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 57.0),
      new WristPreset(m_wrist, 53.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    m_bottomScoreConePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new WristPreset(m_wrist, 0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    m_driveTrain.setDefaultCommand(m_arcadeDrive);
    m_shoulder.setDefaultCommand(m_shoulderMaintain);
    m_telescope.setDefaultCommand(m_telescopeMaintain);
    m_wrist.setDefaultCommand(m_wristMaintain);

    SmartDashboard.putData(m_driveTrain);
    SmartDashboard.putData(m_turret);
    SmartDashboard.putData(m_shoulder);
    SmartDashboard.putData(m_wrist);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_limelight);
    SmartDashboard.putData(m_telescope);

    // Configures controller and joystick bindings
    configureBindings();
  }
  
  private void configureBindings() {
    m_driveStraightTrigger.whileTrue(m_driveStraight).onFalse(m_driveStop);

    m_telescopeControlTrigger.whileTrue(m_telescopeControl).onFalse(m_telescopeStop);
    m_turretControlTrigger.whileTrue(m_turretControl).onFalse(m_turretStop);
    m_shoulderUpTrigger.whileTrue(m_shoulderUp).onFalse(m_shoulderStop);
    m_shoulderDownTrigger.whileTrue(m_shoulderDown).onFalse(m_shoulderStop);
    m_wristUpTrigger.whileTrue(m_wristUp).onFalse(m_wristStop);
    m_wristDownTrigger.whileTrue(m_wristDown).onFalse(m_wristStop);

    m_turretTrackingTrigger.whileTrue(m_turretTracking);

    m_doublePlayerStationCubePresetTrigger.whileTrue(m_doublePlayerStationCubePreset).onFalse(m_homePreset);
    m_singlePlayerStationCubePresetTrigger.whileTrue(m_singlePlayerStationCubePreset).onFalse(m_homePreset);
    m_floorCubePresetTrigger.whileTrue(m_floorCubePreset).onFalse(m_homePreset);

    m_doublePlayerStationConePresetTrigger.whileTrue(m_doublePlayerStationConePreset).onFalse(m_homePreset);
    m_singlePlayerStationConePresetTrigger.whileTrue(m_singlePlayerStationConePreset).onFalse(m_homePreset);
    m_floorConePresetTrigger.whileTrue(m_floorConePreset).onFalse(m_homePreset);

    m_topScoreCubePresetTrigger.whileTrue(m_topScoreCubePreset).onFalse(m_homePreset);
    m_middleScoreCubePresetTrigger.whileTrue(m_middleScoreCubePreset).onFalse(m_homePreset);
    m_bottomScoreCubePresetTrigger.whileTrue(m_bottomScoreCubePreset).onFalse(m_homePreset);

    m_topScoreCubePresetTrigger.and(m_releaseTrigger).whileTrue(m_releaseCube).onFalse(m_intakeStop);
    m_middleScoreCubePresetTrigger.and(m_releaseTrigger).whileTrue(m_releaseCube).onFalse(m_intakeStop);
    m_bottomScoreCubePresetTrigger.and(m_releaseTrigger).whileTrue(m_releaseCube).onFalse(m_intakeStop);

    m_topScoreConePresetTrigger.whileTrue(m_topScoreConePreset).onFalse(m_homePreset);
    m_middleScoreConePresetTrigger.whileTrue(m_middleScoreConePreset).onFalse(m_homePreset);
    m_bottomScoreConePresetTrigger.whileTrue(m_bottomScoreConePreset).onFalse(m_homePreset);

    m_topScoreConePresetTrigger.and(m_releaseTrigger).whileTrue(m_releaseCone).onFalse(m_intakeStop);
    m_middleScoreConePresetTrigger.and(m_releaseTrigger).whileTrue(m_releaseCone).onFalse(m_intakeStop);
    m_bottomScoreConePresetTrigger.and(m_releaseTrigger).whileTrue(m_releaseCone).onFalse(m_intakeStop);
  }

  public void updateDashboard() {
    // SmartDashboard.putNumber("Joystick Y", m_operatorJoystick.getY());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

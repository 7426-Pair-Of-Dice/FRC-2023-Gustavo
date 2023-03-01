// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Commands.*;

import java.util.Map;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

  // Input
  private static CommandXboxController m_driverController;
  private static CommandJoystick m_operatorJoystick;

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

  private static Trigger m_turretLeftTrigger;
  private static Trigger m_turretRightTrigger;
  private static Trigger m_shoulderControlTrigger;
  private static Trigger m_telescopeForwardTrigger;
  private static Trigger m_telescopeBackwardTrigger;
  private static Trigger m_wristForwardTrigger;
  private static Trigger m_wristBackwardTrigger;

  // Subsystems
  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Shoulder m_shoulder;
  private static Wrist m_wrist;
  private static Intake m_intake;
  private static Telescope m_telescope;
  private static Limelight m_limelight;

  // Commands
  private static RunCommand m_tankDrive;
  private static RunCommand m_arcadeDrive;

  private static Command m_driveSelector;

  private static SequentialCommandGroup m_homePreset;

  private static RunCommand m_turretLeft;
  private static RunCommand m_turretRight;
  private static RunCommand m_shoulderControl;
  private static RunCommand m_telescopeForward;
  private static RunCommand m_telescopeBackward;
  private static RunCommand m_wristForward;
  private static RunCommand m_wristBackward;

  private static InstantCommand m_turretStop;
  private static InstantCommand m_shoulderStop;
  private static InstantCommand m_telescopeStop;
  private static InstantCommand m_wristStop;

  private static RunCommand m_shoulderMaintain;
  private static RunCommand m_telescopeMaintain;
  private static RunCommand m_wristMaintain;

  private static ParallelCommandGroup m_doublePlayerStationCubePreset;
  private static ParallelCommandGroup m_singlePlayerStationCubePreset;
  private static ParallelCommandGroup m_floorCubePreset;

  private static ParallelCommandGroup m_doublePlayerStationConePreset;
  private static ParallelCommandGroup m_singlePlayerStationConePreset;
  private static ParallelCommandGroup m_floorConePreset;

  private static SequentialCommandGroup m_topScoreCubePreset;
  private static SequentialCommandGroup m_middleScoreCubePreset;
  private static SequentialCommandGroup m_bottomScoreCubePreset;

  private static SequentialCommandGroup m_topScoreConePreset;
  private static SequentialCommandGroup m_middleScoreConePreset;
  private static SequentialCommandGroup m_bottomScoreConePreset;


  public RobotContainer() {
    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorJoystick = new CommandJoystick(Constants.Input.kOperatorJoystickId);

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

    m_turretLeftTrigger = m_operatorJoystick.povLeft();
    m_turretRightTrigger = m_operatorJoystick.povRight();
    m_shoulderControlTrigger = m_operatorJoystick.axisGreaterThan(AxisType.kY.value, 0.05).or(m_operatorJoystick.axisLessThan(AxisType.kY.value, -0.05));
    m_telescopeForwardTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId).negate());
    m_telescopeBackwardTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId));
    m_wristForwardTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId).negate());
    m_wristBackwardTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId));

    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_shoulder = new Shoulder();
    m_wrist = new Wrist();
    m_intake = new Intake();
    m_telescope = new Telescope();
    m_limelight = new Limelight();

    // Commands
    m_tankDrive = new RunCommand(() -> m_driveTrain.tankDrive(-m_driverController.getLeftY() * 0.8, -m_driverController.getRightY() * 0.8), m_driveTrain);
    m_arcadeDrive = new RunCommand(() -> m_driveTrain.arcadeDrive(-m_driverController.getLeftY() * 0.8, m_driverController.getRightX() * 0.8), m_driveTrain);

    m_driveSelector = new SelectCommand(
      Map.ofEntries(
        Map.entry(Drivetrain.DriveType.Tank, m_tankDrive),
        Map.entry(Drivetrain.DriveType.Arcade, m_arcadeDrive)
      ), 
      m_driveTrain::getDriveType
    );

    m_homePreset = new SequentialCommandGroup(
      new InstantCommand(() -> m_intake.stopIntake(), m_intake),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new ShoulderPreset(m_shoulder, 0)
    );

    m_turretLeft = new RunCommand(() -> m_turret.setPercentOutput(0.5), m_turret);
    m_turretRight = new RunCommand(() -> m_turret.setPercentOutput(-0.5), m_turret);
    m_shoulderControl = new RunCommand(() -> m_shoulder.setPercentOutput(-m_operatorJoystick.getY() * 0.5), m_shoulder);
    m_telescopeForward = new RunCommand(() -> m_telescope.setPercentOutput(1.0), m_telescope);
    m_telescopeBackward = new RunCommand(() -> m_telescope.setPercentOutput(-1.0), m_telescope);
    m_wristForward = new RunCommand(() -> m_wrist.setPercentOutput(0.25), m_wrist);
    m_wristBackward = new RunCommand(() -> m_wrist.setPercentOutput(-0.25), m_wrist);

    m_turretStop = new InstantCommand(() -> m_turret.stop(), m_turret);
    m_shoulderStop = new InstantCommand(() -> m_shoulder.stop(), m_shoulder);
    m_telescopeStop = new InstantCommand(() -> m_telescope.stop(), m_telescope);
    m_wristStop = new InstantCommand(() -> m_wrist.stop(), m_wrist);

    m_shoulderMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_shoulder);
    m_telescopeMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_telescope);
    m_wristMaintain = new RunCommand(() -> m_wrist.setLastPosition(), m_wrist);

    // Cube grabbing presets
    m_doublePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake),
      new SequentialCommandGroup(
        new ShoulderPreset(m_shoulder, 62.0),
        new TelescopePreset(m_telescope, 0.0),
        new WristPreset(m_wrist, 124.0)
      )
    );
    m_singlePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake),
      new SequentialCommandGroup(
        new ShoulderPreset(m_shoulder, 0),
        new TelescopePreset(m_telescope, 0),
        new WristPreset(m_wrist, 0)
      )
    );
    m_floorCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake),
      new SequentialCommandGroup(
        new ShoulderPreset(m_shoulder, 0),
        new TelescopePreset(m_telescope, 0),
        new WristPreset(m_wrist, 115.0)
      )
    );

    // Cone grabbing presets
    m_doublePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake),
      new SequentialCommandGroup(
        new ShoulderPreset(m_shoulder, 53.0),
        new TelescopePreset(m_telescope, 0),
        new WristPreset(m_wrist, 43.0)
      )
    );
    m_singlePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake),
      new SequentialCommandGroup(
        new ShoulderPreset(m_shoulder, 0),
        new TelescopePreset(m_telescope, 0),
        new WristPreset(m_wrist, 0)
      )
    );
    m_floorConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake),
      new SequentialCommandGroup(
        new ShoulderPreset(m_shoulder, 0),
        new TelescopePreset(m_telescope, 0),
        new WristPreset(m_wrist, 50.0)
      )
    );

    // Cube scoring presets
    m_topScoreCubePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new RunCommand(() -> m_intake.releaseCube(), m_intake)
    );
    m_middleScoreCubePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new RunCommand(() -> m_intake.releaseCube(), m_intake)
    );
    m_bottomScoreCubePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new RunCommand(() -> m_intake.releaseCube(), m_intake)
    );

    // Cone scoring presets
    m_topScoreConePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new RunCommand(() -> m_intake.releaseCone(), m_intake)
    );
    m_middleScoreConePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new RunCommand(() -> m_intake.releaseCone(), m_intake)
    );
    m_bottomScoreConePreset = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 0),
      new TelescopePreset(m_telescope, 0),
      new WristPreset(m_wrist, 0),
      new RunCommand(() -> m_intake.releaseCone(), m_intake)
    );

    m_driveTrain.setDefaultCommand(m_driveSelector);
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
    m_doublePlayerStationCubePresetTrigger.whileTrue(m_doublePlayerStationCubePreset).onFalse(m_homePreset);
    m_singlePlayerStationCubePresetTrigger.whileTrue(m_singlePlayerStationCubePreset).onFalse(m_homePreset);
    m_floorCubePresetTrigger.whileTrue(m_floorCubePreset).onFalse(m_homePreset);

    m_doublePlayerStationConePresetTrigger.whileTrue(m_doublePlayerStationConePreset).onFalse(m_homePreset);
    m_singlePlayerStationConePresetTrigger.whileTrue(m_singlePlayerStationConePreset).onFalse(m_homePreset);
    m_floorConePresetTrigger.whileTrue(m_floorConePreset).onFalse(m_homePreset);

    m_topScoreCubePresetTrigger.whileTrue(m_topScoreCubePreset).onFalse(m_homePreset);
    m_middleScoreCubePresetTrigger.whileTrue(m_middleScoreCubePreset).onFalse(m_homePreset);
    m_bottomScoreCubePresetTrigger.whileTrue(m_bottomScoreCubePreset).onFalse(m_homePreset);

    m_topScoreConePresetTrigger.whileTrue(m_topScoreConePreset).onFalse(m_homePreset);
    m_middleScoreConePresetTrigger.whileTrue(m_middleScoreConePreset).onFalse(m_homePreset);
    m_bottomScoreConePresetTrigger.whileTrue(m_bottomScoreConePreset).onFalse(m_homePreset);

    m_turretLeftTrigger.whileTrue(m_turretLeft).onFalse(m_turretStop);
    m_turretRightTrigger.whileTrue(m_turretRight).onFalse(m_turretStop);
    m_shoulderControlTrigger.whileTrue(m_shoulderControl).onFalse(m_shoulderStop);
    m_telescopeForwardTrigger.whileTrue(m_telescopeForward).onFalse(m_telescopeStop);
    m_telescopeBackwardTrigger.whileTrue(m_telescopeBackward).onFalse(m_telescopeStop);
    m_wristForwardTrigger.whileTrue(m_wristForward).onFalse(m_wristStop);
    m_wristBackwardTrigger.whileTrue(m_wristBackward).onFalse(m_wristStop);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Joystick Heading", m_operatorJoystick.getDirectionDegrees());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

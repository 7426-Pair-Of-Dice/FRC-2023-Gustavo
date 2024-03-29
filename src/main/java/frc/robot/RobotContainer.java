// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Subsystems.LED.BottomLEDState;
import frc.robot.Subsystems.LED.TopLEDState;
import frc.robot.Commands.*;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class RobotContainer {

  public enum Alliance {
    BLUE,
    RED
  }

  public enum RobotState {
    TELEOP,
    AUTONOMOUS,
    TEST,
    DISABLED
  }

  private static RobotState m_robotState;

  private static Timer m_matchTimer;

  // Input
  private static CommandXboxController m_driverController;
  private static CommandJoystick m_operatorJoystick;
  
  private static Trigger m_driveSlowTrigger;
  private static Trigger m_driveSlowerTrigger;
  private static Trigger m_driveInvertTrigger;

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

  private static Trigger m_turretControlTrigger;
  private static Trigger m_turretSlowLeftTrigger;
  private static Trigger m_turretSlowRightTrigger;

  private static Trigger m_releaseTrigger;

  private static Trigger m_turretConeTopTrackingTrigger;
  private static Trigger m_turretConeBottomTrackingTrigger;
  private static Trigger m_turretCubeTrackingTrigger;

  private static Trigger m_shoulderHomeTrigger;

  private static Trigger m_cubeWantedTrigger;
  private static Trigger m_coneWantedTrigger;

  // Subsystems
  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Shoulder m_shoulder;
  private static Wrist m_wrist;
  private static Intake m_intake;
  private static Limelight m_limelight;
  private static LED m_led;

  // Commands
  private static RunCommand m_arcadeDrive;
  private static InstantCommand m_arcadeDriveNormal;
  private static InstantCommand m_arcadeDriveSlow;
  private static InstantCommand m_arcadeDriveSlower;
  private static InstantCommand m_arcadeDriveInvert;

  private static SequentialCommandGroup m_homePreset;

  private static ParallelCommandGroup m_shoulderHomePreset;

  private static RunCommand m_turretControl;
  private static RunCommand m_turretSlowLeft;
  private static RunCommand m_turretSlowRight;
  private static RunCommand m_releaseCube;
  private static RunCommand m_releaseCone;

  private static TurretTracking m_turretTrackingConeTop;
  private static TurretTracking m_turretTrackingConeBottom;
  private static TurretTracking m_turretTrackingCube;

  private static InstantCommand m_turretStop;
  private static InstantCommand m_shoulderStop;
  private static InstantCommand m_intakeStop;
  private static InstantCommand m_limelightReset;

  private static RunCommand m_shoulderMaintain;
  private static RunCommand m_wristMaintain;
  private static RunCommand m_intakeMaintain;

  private static ParallelCommandGroup m_doublePlayerStationCubePreset;
  private static ParallelCommandGroup m_singlePlayerStationCubePreset;
  private static ParallelCommandGroup m_floorCubePreset;

  private static ParallelCommandGroup m_doublePlayerStationConePreset;
  private static ParallelCommandGroup m_singlePlayerStationConePreset;
  private static ParallelCommandGroup m_floorConePreset;

  private static ParallelCommandGroup m_topScoreCubePreset;
  private static ParallelCommandGroup m_middleScoreCubePreset;
  private static ParallelCommandGroup m_bottomScoreCubePreset;

  private static ParallelCommandGroup m_topScoreConePreset;
  private static ParallelCommandGroup m_middleScoreConePreset;
  private static ParallelCommandGroup m_bottomScoreConePreset;

  private static PrintCommand m_defaultAuto;
  private static SelectCommand m_oneConeBalance;
  private static SelectCommand m_oneConeTaxiBalance;
  private static SelectCommand m_oneConeOneCube;
  private static SelectCommand m_oneConeBackwards;
  private static SequentialCommandGroup m_oneCone;
  private static SequentialCommandGroup m_test;

  private static SendableChooser<Alliance> m_allianceChooser;
  private static SendableChooser<Command> m_autoChooser;

  private static WrapperCommand m_ledStateManager;

  public RobotContainer() {

    m_robotState = RobotState.DISABLED;

    m_matchTimer = new Timer();

    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorJoystick = new CommandJoystick(Constants.Input.kOperatorJoystickId);

    m_driveSlowTrigger = m_driverController.leftBumper();
    m_driveSlowerTrigger = m_driverController.leftTrigger();
    m_driveInvertTrigger = m_driverController.rightBumper();

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

    m_turretControlTrigger = m_operatorJoystick.axisGreaterThan(AxisType.kZ.value, 0.1).or(m_operatorJoystick.axisLessThan(AxisType.kZ.value, -0.1));
    m_turretSlowLeftTrigger = m_operatorJoystick.povLeft();
    m_turretSlowRightTrigger = m_operatorJoystick.povRight();

    m_releaseTrigger = m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId);

    m_turretConeTopTrackingTrigger = m_driverController.y();
    m_turretConeBottomTrackingTrigger = m_driverController.a();
    m_turretCubeTrackingTrigger = m_driverController.x();

    m_shoulderHomeTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterMiddleButtonId);

    m_cubeWantedTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId);
    m_coneWantedTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId);

    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_shoulder = new Shoulder();
    m_wrist = new Wrist();
    m_intake = new Intake();
    m_limelight = new Limelight();
    m_led = new LED();

    // Commands
    m_arcadeDrive = new RunCommand(
      () -> {
        double leftY = -m_driverController.getLeftY();
        double rightX = m_driverController.getRightX();

        double speedInput;
        double rotationInput;

        double shoulderAngle = m_shoulder.getAngle();

        if (shoulderAngle > 60.0) {
          speedInput = leftY * 0.2;
          rotationInput = rightX * 0.12;
        } else if (shoulderAngle > 30.0) {
          speedInput = leftY * 0.4;
          rotationInput = rightX * 0.4;
        } else if (shoulderAngle > 20.0) {
          speedInput = leftY * 0.75;
          rotationInput = rightX * 0.5;
        } else {
          speedInput = leftY;
          rotationInput = rightX * 0.4;
        }

        m_driveTrain.arcadeDrive(speedInput, rotationInput);
      }, 
      m_driveTrain
    );
    
    m_arcadeDriveNormal = new InstantCommand(() -> m_driveTrain.setMultiplier(1.0), m_driveTrain);
    m_arcadeDriveSlow = new InstantCommand(() -> m_driveTrain.setMultiplier(0.3), m_driveTrain);
    m_arcadeDriveSlower = new InstantCommand(() -> m_driveTrain.setMultiplier(0.2), m_driveTrain);
    m_arcadeDriveInvert = new InstantCommand(() -> m_driveTrain.invertSpeed(), m_driveTrain);

    m_turretControl = new RunCommand(() -> m_turret.setPercentOutput(-m_operatorJoystick.getZ() * 0.5), m_turret); //Was full speed
    m_turretSlowLeft = new RunCommand(() -> m_turret.setPercentOutput(0.15), m_turret);
    m_turretSlowRight = new RunCommand(() -> m_turret.setPercentOutput(-0.15), m_turret);
    m_releaseCube = new RunCommand(() -> m_intake.releaseCube(), m_intake);
    m_releaseCone = new RunCommand(() -> m_intake.releaseCone(), m_intake);

    m_turretTrackingConeTop = new TurretTracking(m_turret, m_limelight, Constants.Limelight.kRetroReflectiveTopPipeline);
    m_turretTrackingConeBottom = new TurretTracking(m_turret, m_limelight, Constants.Limelight.kRetroReflectiveBottomPipeline);
    m_turretTrackingCube = new TurretTracking(m_turret, m_limelight, Constants.Limelight.kAprilTagPipeline);

    m_turretStop = new InstantCommand(() -> m_turret.stop(), m_turret);
    m_shoulderStop = new InstantCommand(() -> m_shoulder.stop(), m_shoulder);
    m_intakeStop = new InstantCommand(() -> m_intake.stop(), m_intake);
    m_limelightReset = new InstantCommand(() -> m_limelight.setPipeline(0), m_limelight);

    m_shoulderMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_shoulder);
    m_wristMaintain = new RunCommand(() -> m_wrist.setLastPosition(), m_wrist);
    m_intakeMaintain = new RunCommand(
      () -> {
        if (m_intake.getCubeDetected()) {
          m_intake.intakeCubeSlow();
        } else if (m_intake.getConeDetected()) {
          m_intake.intakeConeSlow();
        } else {
          m_intake.stop();
        }
      }, 
      m_intake
    );

    m_homePreset = new SequentialCommandGroup(
      new WristPreset(m_wrist, 0.0, 5.0),
      new ShoulderPreset(m_shoulder, 10.0, 5.0),
      new TurretPreset(m_turret, 0.0, 5.0)
    );

    m_shoulderHomePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 0.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0)
    );

    // Cube grabbing presets
    m_doublePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 75.0, 5.0),
      new WristPreset(m_wrist, 118.0, 5.0)
    );
    m_singlePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 27.0, 5.0),
      new WristPreset(m_wrist, 0, 34.0)
    );
    m_floorCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 16.0, 5.0),
      new WristPreset(m_wrist, 100.0, 5.0)
    );

    // Cone grabbing presets
    m_doublePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake).until(m_intake::getConeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 64.0, 5.0),
      new WristPreset(m_wrist, 43.0, 5.0)
    );
    m_singlePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake).until(m_intake::getConeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 43.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0)
    );
    m_floorConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake).until(m_intake::getConeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 14.0, 5.0),
      new WristPreset(m_wrist, 36.0, 5.0)
    );

    // Cube scoring presets
    m_topScoreCubePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 78.0, 5.0),
      new WristPreset(m_wrist, 110.0, 5.0)
    );
    m_middleScoreCubePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 58.0, 5.0),
      new WristPreset(m_wrist, 104.0, 5.0)
    );
    m_bottomScoreCubePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 18.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0)
    );

    // Cone scoring presets
    m_topScoreConePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 95.0, 5.0),
      new WristPreset(m_wrist, 90.0, 5.0)
    );
    m_middleScoreConePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 75.0 , 5.0),
      new WristPreset(m_wrist, 110.0, 5.0)
    );
    m_bottomScoreConePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 18.0, 5.0),
      new WristPreset(m_wrist, 0, 5.0)
    );

    m_ledStateManager = new RunCommand(
      () -> {
        if (m_robotState == RobotState.DISABLED) {
          m_led.setTopLEDState(TopLEDState.IDLE);
          m_led.setBottomLEDState(BottomLEDState.IDLE);
          return;
        }

        if (m_intake.getCubeDetected()) {
          m_led.setTopLEDState(TopLEDState.CUBE_HAVE);
        } else if (m_cubeWantedTrigger.getAsBoolean()) {
          m_led.setTopLEDState(TopLEDState.CUBE_WANTED);
        } else if (m_intake.getConeDetected()) {
          m_led.setTopLEDState(TopLEDState.CONE_HAVE);
        } else if (m_coneWantedTrigger.getAsBoolean()) {
          m_led.setTopLEDState(TopLEDState.CONE_WANTED);
        } else {
          m_led.setTopLEDState(TopLEDState.IDLE);
        }

        if (m_matchTimer.get() > 135) {
          m_led.setBottomLEDState(BottomLEDState.IDLE);
        } else if (m_matchTimer.get() > 120) {
          m_led.setBottomLEDState(BottomLEDState.FIFTEEN_SECONDS_LEFT);
        } else if (m_matchTimer.get() > 105) {
          m_led.setBottomLEDState(BottomLEDState.THIRTY_SECONDS_LEFT);
        } else if (m_matchTimer.get() > 75 && m_matchTimer.get() < 78) {
          m_led.setBottomLEDState(BottomLEDState.MINUTE_LEFT);
        }
      }, 
      m_led
    ).ignoringDisable(true);

    m_driveTrain.setDefaultCommand(m_arcadeDrive);
    m_shoulder.setDefaultCommand(m_shoulderMaintain);
    m_wrist.setDefaultCommand(m_wristMaintain);
    m_intake.setDefaultCommand(m_intakeMaintain);
    m_led.setDefaultCommand(m_ledStateManager);

    // Autonomous Commands
    m_defaultAuto = new PrintCommand("Default Auto");

    m_oneConeBalance = new SelectCommand(
      Map.ofEntries(
        Map.entry(
          Alliance.BLUE, 
          new SequentialCommandGroup(
            new ShoulderPreset(m_shoulder, 105.0, 5.0),
            new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 96.0, 5.0),
            new WaitCommand(0.2),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WaitCommand(0.2),
            new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 0.0, 5.0),
            new DriveStraight(m_driveTrain, -0.45).until(m_driveTrain::isTipped),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new RunCommand(() -> m_driveTrain.arcadeDrive(-0.11, 0), m_driveTrain).raceWith(new WaitCommand(2.6)),
            // new WaitCommand(0.5),
            new AutoBalance(m_driveTrain)
          )
        ),
        Map.entry(
          Alliance.RED, 
          new SequentialCommandGroup(
            new ShoulderPreset(m_shoulder, 105.0, 5.0),
            new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 95.0, 5.0),
            new WaitCommand(0.2),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WaitCommand(0.2),
            new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 0.0, 5.0),
            new DriveStraight(m_driveTrain, -0.45).until(m_driveTrain::isTipped),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new RunCommand(() -> m_driveTrain.arcadeDrive(-0.11, 0), m_driveTrain).raceWith(new WaitCommand(2.6)),
            // new WaitCommand(0.5),
            new AutoBalance(m_driveTrain)
          )
        )
      ),
      this::getAlliance
    );

    m_oneConeTaxiBalance = new SelectCommand(
      Map.ofEntries(
        Map.entry(
          Alliance.BLUE, 
          new SequentialCommandGroup(
            new ShoulderPreset(m_shoulder, 105.0, 5.0),
            new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 96.0, 5.0),
            new WaitCommand(0.2),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WaitCommand(0.2),
            new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new ParallelCommandGroup(
              new WristPreset(m_wrist, 0.0, 5.0),
              new ShoulderPreset(m_shoulder, 0.0, 5.0),
              new DriveStraight(m_driveTrain, -0.35).until(m_driveTrain::isTipped)
            ),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new RunCommand(() -> m_driveTrain.arcadeDrive(-0.25, 0), m_driveTrain).raceWith(new WaitCommand(1.8)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WaitCommand(0.8),
            new DriveStraight(m_driveTrain, 0.35).until(m_driveTrain::isTipped),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new RunCommand(() -> m_driveTrain.arcadeDrive(0.11, 0), m_driveTrain).raceWith(new WaitCommand(2.2)),
            //new WaitCommand(0.5),
            new AutoBalance(m_driveTrain)
          )
        ),
        Map.entry(
          Alliance.RED, 
          new SequentialCommandGroup(
            new ShoulderPreset(m_shoulder, 105.0, 5.0),
            new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 95.0, 5.0),
            new WaitCommand(0.2),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WaitCommand(0.2),
            new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new ParallelCommandGroup(
              new WristPreset(m_wrist, 0.0, 5.0),
              new ShoulderPreset(m_shoulder, 0.0, 5.0),
              new DriveStraight(m_driveTrain, -0.35).until(m_driveTrain::isTipped)
            ),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new RunCommand(() -> m_driveTrain.arcadeDrive(-0.25, 0), m_driveTrain).raceWith(new WaitCommand(1.8)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WaitCommand(0.8),
            new DriveStraight(m_driveTrain, 0.35).until(m_driveTrain::isTipped),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new RunCommand(() -> m_driveTrain.arcadeDrive(0.11, 0), m_driveTrain).raceWith(new WaitCommand(2.2)),
            //new WaitCommand(0.5),
            new AutoBalance(m_driveTrain)
          )
        )
      ),
      this::getAlliance
    );

    m_oneConeOneCube = new SelectCommand(
      Map.ofEntries(
        Map.entry(
          Alliance.BLUE, 
          new SequentialCommandGroup(
            new ParallelCommandGroup(
              new ShoulderPreset(m_shoulder, 12.0, 5.0),
              new TurretPreset(m_turret, -20.0, 5.0)
            ),
            new ShoulderPreset(m_shoulder, 73.0, 5.0),
            new WristPreset(m_wrist, 108.0, 5.0),
            new WaitCommand(0.075),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new ParallelCommandGroup(
              new WristPreset(m_wrist, 0.0, 5.0),
              new ShoulderPreset(m_shoulder, 15.0, 5.0)
            ),
            new ParallelCommandGroup(
              new DriveToDistance(m_driveTrain, Units.inchesToMeters(-183.0), Units.inchesToMeters(2.0)),
              new TurretPreset(m_turret, -180.0, 5.0)
            ),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new ShoulderPreset(m_shoulder, 15.0, 5.0),
            new WristPreset(m_wrist, 103.0, 5.0),
            new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 15.0, 5.0),
            new ParallelCommandGroup(
              new DriveToDistance(m_driveTrain, Units.inchesToMeters(180.0), Units.inchesToMeters(2.0)),
              new TurretPreset(m_turret, 7.0, 5.0)
            ),
            new ShoulderPreset(m_shoulder, 58.0, 5.0),
            new WristPreset(m_wrist, 104.0, 5.0),
            new RunCommand(() -> m_intake.releaseCube(), m_intake).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 18.0, 5.0)
          )
        ),
        Map.entry(
          Alliance.RED, 
          new SequentialCommandGroup(
            new ParallelCommandGroup(
              new ShoulderPreset(m_shoulder, 12.0, 5.0),
              new TurretPreset(m_turret, 25.0, 5.0)
            ),
            new ShoulderPreset(m_shoulder, 73.0, 5.0),
            new WristPreset(m_wrist, 108.0, 5.0),
            new WaitCommand(0.075),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new ParallelCommandGroup(
              new WristPreset(m_wrist, 0.0, 5.0),
              new ShoulderPreset(m_shoulder, 15.0, 5.0)
            ),
            new ParallelCommandGroup(
              new DriveToDistance(m_driveTrain, Units.inchesToMeters(-183.0), Units.inchesToMeters(2.0)),
              new TurretPreset(m_turret, 180.0, 5.0)
            ),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new ShoulderPreset(m_shoulder, 15.0, 5.0),
            new WristPreset(m_wrist, 103.0, 5.0),
            new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 15.0, 5.0),
            new ParallelCommandGroup(
              new DriveToDistance(m_driveTrain, Units.inchesToMeters(180.0), Units.inchesToMeters(2.0)),
              new TurretPreset(m_turret, -4.0, 5.0)
            ),
            new ShoulderPreset(m_shoulder, 58.0, 5.0),
            new WristPreset(m_wrist, 104.0, 5.0),
            new RunCommand(() -> m_intake.releaseCube(), m_intake).raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 18.0, 5.0)
          )
        )
      ),
      this::getAlliance
    );
 
    m_oneConeBackwards = new SelectCommand(
      Map.ofEntries(
        Map.entry(
          Alliance.BLUE, 
          new SequentialCommandGroup(
            new ShoulderPreset(m_shoulder, 15.0, 5.0),
            new TurretPreset(m_turret, 5.0, 5.0),  
            new ShoulderPreset(m_shoulder, 105.0, 5.0),
            new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.25)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 95.0, 5.0),
            new WaitCommand(0.2),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WaitCommand(0.2),
            new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.35)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 18.0, 5.0),
            new DriveStraight(m_driveTrain, -0.5).raceWith(new WaitCommand(3.0)),
            new TurretPreset(m_turret, -180, 5.0),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain)
          )
        ),
        Map.entry(
          Alliance.RED,
          new SequentialCommandGroup(
            new ShoulderPreset(m_shoulder, 15.0, 5.0),
            new TurretPreset(m_turret, -5.0, 5.0),
            new ShoulderPreset(m_shoulder, 105.0, 5.0),
            new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.25)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 95.0, 5.0),
            new WaitCommand(0.2),
            new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
            new InstantCommand(() -> m_intake.stop(), m_intake),
            new WaitCommand(0.2),
            new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.35)),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
            new WristPreset(m_wrist, 0.0, 5.0),
            new ShoulderPreset(m_shoulder, 18.0, 5.0),
            new DriveStraight(m_driveTrain, -0.5).raceWith(new WaitCommand(3.0)),
            new TurretPreset(m_turret, 180, 5.0),
            new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain)
          )
        )
      ), 
      this::getAlliance
    );

    m_oneCone = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 105.0, 5.0),
      new DriveStraight(m_driveTrain, 0.12).raceWith(new WaitCommand(0.3)),
      new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
      new WristPreset(m_wrist, 95.0, 5.0),
      new WaitCommand(0.2),
      new RunCommand(() -> m_intake.releaseCone(), m_intake).raceWith(new WaitCommand(0.5)),
      new InstantCommand(() -> m_intake.stop(), m_intake),
      new WaitCommand(0.2),
      new DriveStraight(m_driveTrain, -0.12).raceWith(new WaitCommand(0.3)),
      new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain),
      new WristPreset(m_wrist, 0.0, 5.0),
      new ShoulderPreset(m_shoulder, 0.0, 5.0)
    );

    m_test = new SequentialCommandGroup(
      new PrintCommand("Test"),
      new DriveToDistance(m_driveTrain, Units.inchesToMeters(100.0), Units.inchesToMeters(2.0)),
      new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain)
    );
    
    // Smart Dashboard 
    m_allianceChooser = new SendableChooser<Alliance>();
    m_allianceChooser.setDefaultOption("Blue", Alliance.BLUE);
    m_allianceChooser.addOption("Red", Alliance.RED);

    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("Do Nothing", m_defaultAuto);
    m_autoChooser.addOption("OneCone-Balance", m_oneConeBalance);
    m_autoChooser.addOption("OneCone-Taxi-Balance", m_oneConeTaxiBalance);
    m_autoChooser.addOption("OneCone-OneCube", m_oneConeOneCube);
    m_autoChooser.addOption("OneCone-Backwards", m_oneConeBackwards);
    m_autoChooser.addOption("OneCone", m_oneCone);
    m_autoChooser.addOption("Test", m_test);

    SmartDashboard.putData("Alliance Chooser", m_allianceChooser);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putData(m_driveTrain);
    SmartDashboard.putData(m_turret);
    SmartDashboard.putData(m_shoulder);
    SmartDashboard.putData(m_wrist);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_limelight);

    // Configures controller and joystick bindings
    configureBindings();
  }
  
  private void configureBindings() {
    m_driveSlowTrigger.and(m_driveSlowerTrigger.negate()).onTrue(m_arcadeDriveSlow).onFalse(m_arcadeDriveNormal);
    m_driveSlowerTrigger.and(m_driveSlowTrigger.negate()).onTrue(m_arcadeDriveSlower).onFalse(m_arcadeDriveNormal);
    m_driveInvertTrigger.onTrue(m_arcadeDriveInvert).onFalse(m_arcadeDriveInvert);

    m_turretControlTrigger.whileTrue(m_turretControl).onFalse(m_turretStop);
    m_turretSlowLeftTrigger.whileTrue(m_turretSlowLeft).onFalse(m_turretStop);
    m_turretSlowRightTrigger.whileTrue(m_turretSlowRight).onFalse(m_turretStop);

    m_turretConeTopTrackingTrigger.and(m_turretCubeTrackingTrigger.negate()).and(m_turretConeBottomTrackingTrigger.negate()).whileTrue(m_turretTrackingConeTop).onFalse(m_turretStop).onFalse(m_limelightReset);
    m_turretConeBottomTrackingTrigger.and(m_turretCubeTrackingTrigger.negate()).and(m_turretConeTopTrackingTrigger.negate()).whileTrue(m_turretTrackingConeBottom).onFalse(m_turretStop).onFalse(m_limelightReset);
    m_turretCubeTrackingTrigger.and(m_turretConeTopTrackingTrigger.negate()).and(m_turretConeBottomTrackingTrigger.negate()).whileTrue(m_turretTrackingCube).onFalse(m_turretStop).onFalse(m_limelightReset);

    m_doublePlayerStationCubePresetTrigger.whileTrue(m_doublePlayerStationCubePreset).onFalse(m_homePreset).onFalse(m_intakeStop);
    m_singlePlayerStationCubePresetTrigger.whileTrue(m_singlePlayerStationCubePreset).onFalse(m_homePreset).onFalse(m_intakeStop);
    m_floorCubePresetTrigger.whileTrue(m_floorCubePreset).onFalse(m_homePreset).onFalse(m_intakeStop);

    m_doublePlayerStationConePresetTrigger.whileTrue(m_doublePlayerStationConePreset).onFalse(m_homePreset).onFalse(m_intakeStop);
    m_singlePlayerStationConePresetTrigger.whileTrue(m_singlePlayerStationConePreset).onFalse(m_homePreset).onFalse(m_intakeStop);
    m_floorConePresetTrigger.whileTrue(m_floorConePreset).onFalse(m_homePreset).onFalse(m_intakeStop);

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

    m_shoulderHomeTrigger.whileTrue(m_shoulderHomePreset).onFalse(m_shoulderStop);
  }

  public void updateDashboard() {}

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public Alliance getAlliance() {
    return m_allianceChooser.getSelected();
  }

  public void setRobotState(RobotState state) {
    m_robotState = state;

    if (m_robotState == RobotState.DISABLED) {
      m_matchTimer.stop();
      m_matchTimer.reset();
    } else if (m_robotState == RobotState.TELEOP) {
      m_matchTimer.start();
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Commands.*;

import java.util.Map;

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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {

  public enum Alliance {
    BLUE,
    RED
  }

  public enum AutoRoutine {
    DEFAULT,
    ONECUBE_BALANCE,
    ONECUBE_TAXI_BALANCE,
    ONECUBE_TAXI
  }

  // Input
  private static CommandXboxController m_driverController;
  private static CommandJoystick m_operatorJoystick;
  
  private static Trigger m_driveSlowTrigger;
  private static Trigger m_driveStraightTrigger;
  private static Trigger m_autoBalanceTrigger;

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

  private static Trigger m_shoulderUpTrigger;
  private static Trigger m_shoulderDownTrigger;
  private static Trigger m_wristUpTrigger;
  private static Trigger m_wristDownTrigger;
  private static Trigger m_releaseTrigger;

  private static Trigger m_turretConeTrackingTrigger;
  private static Trigger m_turretCubeTrackingTrigger;

  private static Trigger m_shoulderHomeTrigger;

  // Subsystems
  private static Drivetrain m_driveTrain;
  private static Turret m_turret;
  private static Shoulder m_shoulder;
  private static Wrist m_wrist;
  private static Intake m_intake;
  private static Limelight m_limelight;

  // Commands
  private static RunCommand m_arcadeDrive;
  private static RunCommand m_arcadeDriveSlow;
  private static DriveStraight m_driveStraight;
  
  private static AutoBalance m_autoBalance;

  private static SequentialCommandGroup m_homePreset;

  private static ShoulderPreset m_shoulderHomePreset;

  private static RunCommand m_shoulderUp;
  private static RunCommand m_shoulderDown;
  private static RunCommand m_wristUp;
  private static RunCommand m_wristDown;
  private static RunCommand m_turretControl;
  private static RunCommand m_turretSlowLeft;
  private static RunCommand m_turretSlowRight;
  private static RunCommand m_releaseCube;
  private static RunCommand m_releaseCone;

  private static TurretTracking m_turretTrackingCone;
  private static TurretTracking m_turretTrackingCube;

  private static InstantCommand m_turretStop;
  private static InstantCommand m_shoulderStop;
  private static InstantCommand m_wristStop;
  private static InstantCommand m_intakeStop;
  private static InstantCommand m_driveStop;
  private static InstantCommand m_limelightReset;

  private static RunCommand m_shoulderMaintain;
  private static RunCommand m_wristMaintain;

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
  private static SequentialCommandGroup m_oneCubeBalance;
  private static SequentialCommandGroup m_oneCubeTaxiBalance;
  private static SequentialCommandGroup m_oneCubeTaxi;

  private static SelectCommand m_autoCommandSelector;

  private static SendableChooser<Alliance> m_allianceChooser;
  private static SendableChooser<AutoRoutine> m_autoChooser;

  public RobotContainer() {
    // Input
    m_driverController = new CommandXboxController(Constants.Input.kDriverControllerId);
    m_operatorJoystick = new CommandJoystick(Constants.Input.kOperatorJoystickId);

    m_driveStraightTrigger = m_driverController.rightTrigger();
    m_driveSlowTrigger = m_driverController.leftTrigger();
    m_autoBalanceTrigger = m_driverController.b();

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

    m_shoulderUpTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId).negate());
    m_shoulderDownTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterLeftButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId));
    m_wristUpTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId));
    m_wristDownTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterRightButtonId).and(m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId).negate());
    m_releaseTrigger = m_operatorJoystick.button(Constants.Input.kJoystickTriggerButtonId);

    m_turretConeTrackingTrigger = m_driverController.y();
    m_turretCubeTrackingTrigger = m_driverController.x();

    m_shoulderHomeTrigger = m_operatorJoystick.button(Constants.Input.kJoystickCenterMiddleButtonId);
    
    // Subsystems
    m_driveTrain = new Drivetrain();
    m_turret = new Turret();
    m_shoulder = new Shoulder();
    m_wrist = new Wrist();
    m_intake = new Intake();
    m_limelight = new Limelight();

    // Commands
    m_arcadeDrive = new RunCommand(
      () -> {
        double leftY = -m_driverController.getLeftY();
        double rightX = m_driverController.getRightX();

        double speedInput;
        double rotationInput;

        double shoulderAngle = m_shoulder.getAngle();

        if (shoulderAngle > 60.0) {
          speedInput = leftY * 0.15;
          rotationInput = rightX * 0.15;
        } else if (shoulderAngle > 30.0) {
          speedInput = leftY * 0.4;
          rotationInput = rightX * 0.4;
        } else if (shoulderAngle > 10.0) {
          speedInput = leftY * 0.75;
          rotationInput = rightX * 0.75;
        } else {
          speedInput = leftY;
          rotationInput = rightX;
        }

        m_driveTrain.arcadeDrive(speedInput, rotationInput);
      }, 
      m_driveTrain
    );

    m_arcadeDriveSlow = new RunCommand(
      () -> m_driveTrain.arcadeDrive(-m_driverController.getLeftY() * 0.3, m_driverController.getRightX() * 0.3),
      m_driveTrain
    );

    m_driveStraight = new DriveStraight(m_driveTrain, m_driverController);

    m_autoBalance = new AutoBalance(m_driveTrain);

    m_turretControl = new RunCommand(() -> m_turret.setPercentOutput(-m_operatorJoystick.getZ()), m_turret);
    m_turretSlowLeft = new RunCommand(() -> m_turret.setPercentOutput(0.15), m_turret);
    m_turretSlowRight = new RunCommand(() -> m_turret.setPercentOutput(-0.15), m_turret);
    m_shoulderUp = new RunCommand(() -> m_shoulder.setPercentOutput(0.5), m_shoulder);
    m_shoulderDown = new RunCommand(() -> m_shoulder.setPercentOutput(-0.5), m_shoulder);
    m_wristUp = new RunCommand(() -> m_wrist.setPercentOutput(-0.25), m_wrist);
    m_wristDown = new RunCommand(() -> m_wrist.setPercentOutput(0.25), m_wrist);
    m_releaseCube = new RunCommand(() -> m_intake.releaseCube(), m_intake);
    m_releaseCone = new RunCommand(() -> m_intake.releaseCone(), m_intake);

    m_turretTrackingCone = new TurretTracking(m_turret, m_limelight, Constants.Limelight.kRetroReflectivePipeline);
    m_turretTrackingCube = new TurretTracking(m_turret, m_limelight, Constants.Limelight.kAprilTagPipeline);

    m_turretStop = new InstantCommand(() -> m_turret.stop(), m_turret);
    m_shoulderStop = new InstantCommand(() -> m_shoulder.stop(), m_shoulder);
    m_wristStop = new InstantCommand(() -> m_wrist.stop(), m_wrist);
    m_intakeStop = new InstantCommand(() -> m_intake.stop(), m_intake);
    m_driveStop = new InstantCommand(() -> m_driveTrain.stop(), m_driveTrain);
    m_limelightReset = new InstantCommand(() -> m_limelight.setPipeline(0), m_limelight);

    m_shoulderMaintain = new RunCommand(() -> m_shoulder.setLastPosition(), m_shoulder);
    m_wristMaintain = new RunCommand(() -> m_wrist.setLastPosition(), m_wrist);

    m_homePreset = new SequentialCommandGroup(
      new TurretPreset(m_turret, 0.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0),
      new ShoulderPreset(m_shoulder, 0.0, 5.0)
    );

    m_shoulderHomePreset = new ShoulderPreset(m_shoulder, 10.0, 5.0);

    // Cube grabbing presets
    m_doublePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 69.0, 5.0),
      new WristPreset(m_wrist, 124.0, 5.0)
    );
    m_singlePlayerStationCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 31.0, 5.0),
      new WristPreset(m_wrist, 0, 34.0)
    );
    m_floorCubePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCube(), m_intake).until(m_intake::getCubeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 14.0, 5.0),
      new WristPreset(m_wrist, 110.0, 5.0)
    );

    // Cone grabbing presets
    m_doublePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake).until(m_intake::getConeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 60.0, 5.0),
      new WristPreset(m_wrist, 65.0, 5.0)
    );
    m_singlePlayerStationConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake).until(m_intake::getConeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 38.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0)
    );
    m_floorConePreset = new ParallelCommandGroup(
      new RunCommand(() -> m_intake.intakeCone(), m_intake).until(m_intake::getConeDetected).andThen(new InstantCommand(() -> m_intake.stop(), m_intake)),
      new ShoulderPreset(m_shoulder, 8.0, 5.0),
      new WristPreset(m_wrist, 51.0, 5.0)
    );

    // Cube scoring presets
    m_topScoreCubePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 71.0, 5.0),
      new WristPreset(m_wrist, 122.0, 5.0)
    );
    m_middleScoreCubePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 65.0, 5.0),
      new WristPreset(m_wrist, 150.0, 5.0)
    );
    m_bottomScoreCubePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 10.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0)
    );

    // Cone scoring presets
    m_topScoreConePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 95.0, 5.0),
      new WristPreset(m_wrist, 125.0, 5.0)
    );
    m_middleScoreConePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 75.0, 5.0),
      new WristPreset(m_wrist, 120.0, 5.0)
    );
    m_bottomScoreConePreset = new ParallelCommandGroup(
      new ShoulderPreset(m_shoulder, 10.0, 5.0),
      new WristPreset(m_wrist, 0.0, 5.0)
    );

    // Autonomous Commands
    m_defaultAuto = new PrintCommand("Default Auto");

    m_oneCubeBalance = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 71.0, 5.0),
      new WristPreset(m_wrist, 122.0, 5.0),
      new RunCommand(() -> m_intake.releaseCube(), m_intake).raceWith(new WaitCommand(1.0)),
      new DriveToDistance(m_driveTrain, Units.inchesToMeters(-24.0), Units.inchesToMeters(2.0)),
      new RunCommand(() -> m_driveTrain.arcadeDrive(-0.5, 0), m_driveTrain).until(m_driveTrain::isTipped),
      new AutoBalance(m_driveTrain)
    );

    m_oneCubeTaxiBalance = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 71.0, 5.0),
      new WristPreset(m_wrist, 122.0, 5.0),
      new RunCommand(() -> m_intake.releaseCube(), m_intake).raceWith(new WaitCommand(1.0)),
      new DriveToDistance(m_driveTrain, Units.inchesToMeters(-24.0), Units.inchesToMeters(2.0)),
      new RunCommand(() -> m_driveTrain.arcadeDrive(-0.5, 0), m_driveTrain).until(m_driveTrain::isTipped),
      new AutoBalance(m_driveTrain),
      new RunCommand(() -> m_driveTrain.arcadeDrive(-0.5, 0), m_driveTrain).raceWith(new WaitCommand(3.0)),
      new RunCommand(() -> m_driveTrain.arcadeDrive(0.5, 0), m_driveTrain).until(m_driveTrain::isTipped),
      new AutoBalance(m_driveTrain)
    );

    m_oneCubeTaxi = new SequentialCommandGroup(
      new ShoulderPreset(m_shoulder, 71.0, 5.0),
      new WristPreset(m_wrist, 122.0, 5.0),
      new RunCommand(() -> m_intake.releaseCube(), m_intake).raceWith(new WaitCommand(1.0)),
      new DriveToDistance(m_driveTrain, Units.inchesToMeters(-72.0), Units.inchesToMeters(2.0))
    );

    m_autoCommandSelector = new SelectCommand(
      Map.ofEntries(
        Map.entry(
          Alliance.BLUE, 
          new SelectCommand(
            Map.ofEntries(
              Map.entry(
                AutoRoutine.DEFAULT,
                m_defaultAuto
              ),
              Map.entry(
                AutoRoutine.ONECUBE_BALANCE,
                m_oneCubeBalance
              ),
              Map.entry(
                AutoRoutine.ONECUBE_TAXI_BALANCE,
                m_oneCubeTaxiBalance
              ),
              Map.entry(
                AutoRoutine.ONECUBE_TAXI,
                m_oneCubeTaxi
              )
            ), 
            this::getAutoRoutine
          )
        ),
        Map.entry(
          Alliance.RED, 
          new SelectCommand(
            Map.ofEntries(
              Map.entry(
                AutoRoutine.DEFAULT,
                m_defaultAuto
              ),
              Map.entry(
                AutoRoutine.ONECUBE_BALANCE,
                m_oneCubeBalance
              ),
              Map.entry(
                AutoRoutine.ONECUBE_TAXI_BALANCE,
                m_oneCubeTaxiBalance
              ),
              Map.entry(
                AutoRoutine.ONECUBE_TAXI,
                m_oneCubeTaxi
              )
            ), 
            this::getAutoRoutine
          )
        )
      ), 
      this::getAlliance
    );

    m_allianceChooser = new SendableChooser<Alliance>();
    m_allianceChooser.setDefaultOption("Blue", Alliance.BLUE);
    m_allianceChooser.addOption("Red", Alliance.RED);

    m_autoChooser = new SendableChooser<AutoRoutine>();
    m_autoChooser.setDefaultOption("Default", AutoRoutine.DEFAULT);
    m_autoChooser.addOption("OneCube-Balance", AutoRoutine.ONECUBE_BALANCE);
    m_autoChooser.addOption("OneCube-Taxi-Balance", AutoRoutine.ONECUBE_TAXI_BALANCE);
    m_autoChooser.addOption("OneCube-Taxi", AutoRoutine.ONECUBE_TAXI);

    m_driveTrain.setDefaultCommand(m_arcadeDrive);
    m_shoulder.setDefaultCommand(m_shoulderMaintain);
    m_wrist.setDefaultCommand(m_wristMaintain);

    SmartDashboard.putData(m_allianceChooser);
    SmartDashboard.putData(m_autoChooser);
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
    m_driveStraightTrigger.whileTrue(m_driveStraight).onFalse(m_driveStop);
    m_driveSlowTrigger.whileTrue(m_arcadeDriveSlow).onFalse(m_driveStop);
    m_autoBalanceTrigger.whileTrue(m_autoBalance).onFalse(m_driveStop);

    m_turretControlTrigger.whileTrue(m_turretControl).onFalse(m_turretStop);
    m_turretSlowLeftTrigger.whileTrue(m_turretSlowLeft).onFalse(m_turretStop);
    m_turretSlowRightTrigger.whileTrue(m_turretSlowRight).onFalse(m_turretStop);
    m_shoulderUpTrigger.whileTrue(m_shoulderUp).onFalse(m_shoulderStop);
    m_shoulderDownTrigger.whileTrue(m_shoulderDown).onFalse(m_shoulderStop);
    m_wristUpTrigger.whileTrue(m_wristUp).onFalse(m_wristStop);
    m_wristDownTrigger.whileTrue(m_wristDown).onFalse(m_wristStop);

    m_turretConeTrackingTrigger.and(m_turretCubeTrackingTrigger.negate()).whileTrue(m_turretTrackingCone).onFalse(m_turretStop).onFalse(m_limelightReset);
    m_turretCubeTrackingTrigger.and(m_turretConeTrackingTrigger.negate()).whileTrue(m_turretTrackingCube).onFalse(m_turretStop).onFalse(m_limelightReset);

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
    return m_autoCommandSelector;
  }

  public Alliance getAlliance() {
    return m_allianceChooser.getSelected();
  }

  public AutoRoutine getAutoRoutine() {
    return m_autoChooser.getSelected();
  }
}

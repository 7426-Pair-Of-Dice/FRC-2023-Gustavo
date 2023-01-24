// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private static Drivetrain m_driveTrain;

  private static DriveTeleop m_driveTeleop;
  private static StartEndCommand m_breakRobot;
  private static AutoBalance m_autoBalance;
  private static DriveStraight m_driveStraight;

  private static CommandXboxController m_xboxController;
  private static Trigger m_aButton;
  private static Trigger m_bButton;
  private static Trigger m_rightTrigger;

  public RobotContainer() {
    // Input
    m_xboxController = new CommandXboxController(0);
    m_aButton = m_xboxController.a();
    m_bButton = m_xboxController.b();
    m_rightTrigger = m_xboxController.rightTrigger();

    // Subsystems
    m_driveTrain = new Drivetrain();

    // Commands
    m_driveTeleop = new DriveTeleop(m_driveTrain, m_xboxController);

    m_breakRobot = new StartEndCommand(() -> m_driveTrain.enableBreak(), () -> m_driveTrain.disableBreak(), m_driveTrain);

    m_autoBalance = new AutoBalance(m_driveTrain);
    
    m_driveStraight = new DriveStraight(m_driveTrain, m_xboxController);

    m_driveTrain.setDefaultCommand(m_driveTeleop);

    // Configures controller and joystick bindings
    configureBindings();
  }
  

  private void configureBindings() {
    m_aButton.whileTrue(m_autoBalance);
    m_bButton.whileTrue(m_breakRobot);
    m_rightTrigger.whileTrue(m_driveStraight);
  }

  public void updateDashboard() {
    SmartDashboard.putData(m_driveTrain);
  }

  public Command getAutonomousCommand() {

    DifferentialDriveVoltageConstraint autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.Drive.ksVolts,
          Constants.Drive.kvVoltSecondsPerMeter,
          Constants.Drive.kaVoltSecondsSquaredPerMeter
        ),
        Constants.Drive.kDriveKinematics,
        Constants.Drive.kMaxVoltage
      );

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.Drive.kMaxSpeedMetersPerSecond, Constants.Drive.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Drive.kDriveKinematics)
        .addConstraint(autoVoltageConstraint);
    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
      new Pose2d(3, 0, new Rotation2d()), 
      config
    );

    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory, 
      m_driveTrain::getPose, 
      new RamseteController(Constants.Drive.kRamseteB, Constants.Drive.kRamseteZeta), 
      new SimpleMotorFeedforward(
        Constants.Drive.ksVolts, 
        Constants.Drive.kvVoltSecondsPerMeter, 
        Constants.Drive.kaVoltSecondsSquaredPerMeter
      ), 
      Constants.Drive.kDriveKinematics, 
      m_driveTrain::getWheelSpeeds, 
      new PIDController(Constants.Drive.kPDriveVel, 0, 0), 
      new PIDController(Constants.Drive.kPDriveVel, 0, 0), 
      m_driveTrain::tankDriveVolts, 
      m_driveTrain
    );

    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class DriveToDistance extends CommandBase {

  private static Drivetrain m_driveTrain;

  private PIDController m_distPidController;
  private PIDController m_anglePidController;

  private double m_inputDistance;
  private double m_setpointDistance;

  private double m_setpointAngle;

  private double m_tolerance;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(Drivetrain driveTrain, double distanceInMeters, double tolerance) {

    m_driveTrain = driveTrain;

    m_distPidController = new PIDController(Constants.DriveToDistanceCommand.kPDist, Constants.DriveToDistanceCommand.kIDist, Constants.DriveToDistanceCommand.kDDist);
    m_anglePidController = new PIDController(Constants.DriveToDistanceCommand.kPAngle, Constants.DriveToDistanceCommand.kIAngle, Constants.DriveToDistanceCommand.kDAngle);

    m_inputDistance = distanceInMeters;

    m_tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setpointDistance = m_driveTrain.getAverageDistance() + m_inputDistance;
    m_setpointAngle = m_driveTrain.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_distPidController.calculate(m_driveTrain.getAverageDistance(), m_setpointDistance);
    speed = speed + (Math.signum(speed) * Constants.DriveToDistanceCommand.kMinCommand);
    double rotation = m_anglePidController.calculate(m_driveTrain.getYaw(), m_setpointAngle);

    m_driveTrain.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getAverageDistance() - m_setpointDistance) < m_tolerance;
  }
}

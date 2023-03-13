// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class RotateToAngle extends CommandBase {

  private static Drivetrain m_driveTrain;

  private PIDController m_pidController;

  private double m_inputAngle;
  private double m_setpointAngle;

  private double m_tolerance;

  /** Creates a new RotateToAngle. */
  public RotateToAngle(Drivetrain driveTrain, double angleInDegrees, double tolerance) {

    m_driveTrain = driveTrain;

    m_pidController = new PIDController(Constants.RotateToAngleCommand.kP, Constants.RotateToAngleCommand.kI, Constants.RotateToAngleCommand.kD);

    m_inputAngle = angleInDegrees;

    m_tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setpointAngle = m_driveTrain.getYaw() + m_inputAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = m_pidController.calculate(m_driveTrain.getYaw(), m_setpointAngle);
    rotation = rotation + (Math.signum(rotation) * Constants.RotateToAngleCommand.kMinCommand);

    m_driveTrain.arcadeDrive(0.0, -rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getYaw() - m_setpointAngle) < m_tolerance;
  }
}

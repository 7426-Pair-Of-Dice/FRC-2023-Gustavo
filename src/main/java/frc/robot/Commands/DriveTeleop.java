// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveTeleop extends CommandBase {
  private static Drivetrain m_driveTrain;

  private static CommandXboxController m_xboxController;

  /** Creates a new DriveTeleop. */
  public DriveTeleop(Drivetrain driveTrain, CommandXboxController xboxController) {
    m_driveTrain = driveTrain;

    m_xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.getDriveType() == Drivetrain.DriveType.Tank) {
      m_driveTrain.tankDrive(-m_xboxController.getLeftY(), -m_xboxController.getRightY());
    } else if (m_driveTrain.getDriveType() == Drivetrain.DriveType.Arcade) {
      m_driveTrain.arcadeDrive(-m_xboxController.getLeftY(), -m_xboxController.getRightX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

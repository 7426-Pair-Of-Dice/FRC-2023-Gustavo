// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class DriveTeleop extends CommandBase {
  
  private final Drivetrain m_driveTrain;

  private final CommandXboxController m_xboxController;

  /** Creates a new DriveTeleop. */
  public DriveTeleop(Drivetrain driveTrain, CommandXboxController xboxController) {
    m_driveTrain = driveTrain;

    m_xboxController = xboxController;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = -m_xboxController.getLeftY();
    double rightSpeed = -m_xboxController.getRightY();

    m_driveTrain.drive(leftSpeed * Constants.Drive.kSpeedDivider, rightSpeed * Constants.Drive.kSpeedDivider);
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

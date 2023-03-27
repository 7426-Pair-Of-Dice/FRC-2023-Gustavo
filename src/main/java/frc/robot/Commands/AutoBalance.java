// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

  private static Drivetrain m_driveTrain;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain driveTrain) {

    m_driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.getPitch() > 14.0) {
      m_driveTrain.arcadeDrive(-0.11, 0);
    } else if (m_driveTrain.getPitch() < -14.0) {
      m_driveTrain.arcadeDrive(0.11, 0);
    } else {
      m_driveTrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

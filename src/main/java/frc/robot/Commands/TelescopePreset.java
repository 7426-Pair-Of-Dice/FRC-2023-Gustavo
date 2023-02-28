// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Telescope;

public class TelescopePreset extends CommandBase {

  private static Telescope m_telescope;

  private static double m_telescopeDistance;

  /** Creates a new ShoulderPreset. */
  public TelescopePreset(Telescope telescope, double telescopeDistance) {

    m_telescope = telescope;

    m_telescopeDistance = telescopeDistance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_telescope.setSetpoint(m_telescopeDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescope.setSetpoint(m_telescopeDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_telescope.atSetpoint();
  }
}

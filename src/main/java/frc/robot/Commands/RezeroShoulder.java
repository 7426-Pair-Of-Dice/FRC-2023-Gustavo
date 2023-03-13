// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shoulder;

public class RezeroShoulder extends CommandBase {

  private static Shoulder m_shoulder;

  /** Creates a new RezeroShoulder. */
  public RezeroShoulder(Shoulder shoulder) {
    m_shoulder = shoulder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulder.disableLimits();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoulder.setPercentOutput(-0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulder.stop();
    m_shoulder.zero();
    m_shoulder.enableLimits();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

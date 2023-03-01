// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Turret;

public class TurretPreset extends CommandBase {

  private static Turret m_turret;

  private double m_turretAngle;

  /** Creates a new ShoulderPreset. */
  public TurretPreset(Turret turret, double turretAngle) {

    m_turret = turret;

    m_turretAngle = turretAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setSetpoint(m_turretAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setPosition(m_turretAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.atSetpoint(5.0);
  }
}

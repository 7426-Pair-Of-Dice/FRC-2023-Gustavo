// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotState;
import frc.robot.Subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

  private static Drivetrain m_driveTrain;

  private Timer m_timer;

  private double m_miliseconds;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain driveTrain) {

    m_driveTrain = driveTrain;

    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = -m_driveTrain.getRoll();

    m_miliseconds = Math.floor(Units.secondsToMilliseconds(m_timer.get()) / 100) * 100;

    if (m_miliseconds % 200 == 0.0) {
      if (Math.abs(m_driveTrain.getRoll()) > 8.1) {
        m_driveTrain.arcadeDrive(0.11 * Math.signum(error), 0);
      } else {
        m_driveTrain.stop();
      }
    } else if (m_miliseconds % 100 == 0.0) {
      m_driveTrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.reset();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

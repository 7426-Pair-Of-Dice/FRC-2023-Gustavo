// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

  private static Drivetrain m_driveTrain;

  private PIDController m_pidController;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain driveTrain) {

    m_driveTrain = driveTrain;

    m_pidController = new PIDController(Constants.AutoBalanceCommand.kP, Constants.AutoBalanceCommand.kI, Constants.AutoBalanceCommand.kD);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_pidController.calculate(m_driveTrain.getPitch(), 0.0);
    speed = Math.max(-Constants.AutoBalanceCommand.kMaxPower, Math.min(Constants.AutoBalanceCommand.kMaxPower, speed));

    m_driveTrain.arcadeDrive(speed, 0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class DriveStraight extends CommandBase {

  private static Drivetrain m_driveTrain;

  private static CommandXboxController m_xboxController;

  private PIDController m_pidController;

  private boolean m_isTeleop;

  private double m_setAngle;

  private double m_speed;

  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain driveTrain, CommandXboxController xboxController) {

    m_driveTrain = driveTrain;

    m_xboxController = xboxController;

    m_pidController = new PIDController(Constants.DriveStraightCommand.kP, Constants.DriveStraightCommand.kI, Constants.DriveStraightCommand.kD);

    m_isTeleop = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  public DriveStraight(Drivetrain driveTrain, double speed) {
    m_driveTrain = driveTrain;

    m_pidController = new PIDController(Constants.DriveStraightCommand.kP, Constants.DriveStraightCommand.kI, Constants.DriveStraightCommand.kD);

    m_isTeleop = false;

    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setAngle = m_driveTrain.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_isTeleop) {
      double speed = -m_xboxController.getLeftY();
      m_driveTrain.arcadeDrive(speed, m_pidController.calculate(m_driveTrain.getYaw(), m_setAngle));
    } else {
      m_driveTrain.arcadeDrive(m_speed, m_pidController.calculate(m_driveTrain.getYaw(), m_setAngle));
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

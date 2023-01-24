// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class DriveStraight extends CommandBase {

  private final Drivetrain m_driveTrain;

  private final CommandXboxController m_xboxController;

  private final Pigeon2 m_gyro;

  private double m_targetAngle;

  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain driveTrain, CommandXboxController xboxController) {
    m_driveTrain = driveTrain;

    m_xboxController = xboxController;

    m_gyro = m_driveTrain.getGyro();

    m_targetAngle = m_gyro.getYaw();

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetAngle = m_gyro.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_xboxController.getLeftY() * Constants.DriveStraight.kSpeedDivider;
    double rotation = (m_targetAngle - m_gyro.getYaw()) * Constants.DriveStraight.kP;

    m_driveTrain.arcadeDrive(speed, rotation);
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

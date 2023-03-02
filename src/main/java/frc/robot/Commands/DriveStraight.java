// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;

public class DriveStraight extends CommandBase {

  private static Drivetrain m_driveTrain;

  private static CommandXboxController m_xboxController;

  private double m_setAngle;

  private double kP = 0.12;

  private double kMaxPower = 0.2;

  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain driveTrain, CommandXboxController xboxController) {

    m_driveTrain = driveTrain;

    m_xboxController = xboxController;

    // m_setAngle = m_driveTrain.getYaw();

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

    double error = m_setAngle - m_driveTrain.getYaw();
    double rotation = Math.max(Math.min(-error * kP, -kMaxPower), kMaxPower);

    System.out.println(error * kP);
    m_driveTrain.arcadeDrive(-m_xboxController.getLeftY(), rotation);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

  private final Drivetrain m_drivetrain;

  private final Pigeon2 m_gyro;

  private final PIDController m_controller;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain driveTrain) {

    m_drivetrain = driveTrain;

    m_gyro = driveTrain.getGyro();

    m_controller = new PIDController(
      Constants.AutoBalanceCommand.kP, 
      Constants.AutoBalanceCommand.kI, 
      Constants.AutoBalanceCommand.kD
    );

    m_controller.setTolerance(Constants.AutoBalanceCommand.kTolerance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = m_controller.calculate(m_gyro.getPitch(), Constants.AutoBalanceCommand.kTargetAngle);
    power = Math.max(-Constants.AutoBalanceCommand.kMaxPower, Math.min(Constants.AutoBalanceCommand.kMaxPower, power));

    m_drivetrain.drive(power, power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

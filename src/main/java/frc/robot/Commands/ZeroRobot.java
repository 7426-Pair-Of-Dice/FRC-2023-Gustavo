// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// Zeroes every subsystem
public class ZeroRobot extends InstantCommand {

  private static Drivetrain m_driveTrain;
  private static Arm m_arm;
  private static Turret m_turret;
  private static Claw m_claw;

  public ZeroRobot(Drivetrain driveTrain, Arm arm, Turret turret, Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_arm = arm;
    m_turret = turret;
    m_claw = claw;

    addRequirements(driveTrain, arm, turret, claw);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.zero();
    m_arm.zero();
    m_turret.zero();
    m_claw.zero();
  }
}

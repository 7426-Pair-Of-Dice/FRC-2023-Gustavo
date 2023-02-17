// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.Arm;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmTeleop extends CommandBase {

  private static Arm m_arm;

  private static CommandXboxController m_xboxController;

  /** Creates a new ArmTeleop. */
  public ArmTeleop(Arm arm, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_arm = arm;

    m_xboxController = xboxController;

    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double armOutput = -m_xboxController.getLeftY() * 0.3;
    double armTelescopeOutput = -m_xboxController.getRightY() * 0.6;


    if (Math.abs(armOutput) > 0.05) {
      m_arm.setArmPercentOutput(armOutput);
    } else {
      m_arm.setArmLastPosition();
    }

    m_arm.setTelescopePercentOutput(armTelescopeOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmPreset extends ParallelCommandGroup {

  private static FunctionalCommand m_shoulderPreset;
  private static FunctionalCommand m_telescopePreset;
  private static FunctionalCommand m_wristPreset;

  /** Creates a new ArmPreset. */
  public ArmPreset(Shoulder shoulder, Telescope telescope, Wrist wrist, double shoulderAngle, double telescopeDistance, double wristAngle) {

    m_shoulderPreset = new FunctionalCommand(
      () -> shoulder.setSetpoint(shoulderAngle),
      () -> shoulder.setPosition(shoulderAngle), 
      interrupted -> {},
      () -> shoulder.atSetpoint(),
      shoulder
    );

    m_telescopePreset = new FunctionalCommand(
      () -> telescope.setSetpoint(telescopeDistance),
      () -> telescope.setPosition(telescopeDistance), 
      interrupted -> {},
      () -> telescope.atSetpoint(),
      telescope
    );

    m_wristPreset = new FunctionalCommand(
      () -> wrist.setSetpoint(wristAngle),
      () -> wrist.setPosition(wristAngle), 
      interrupted -> {},
      () -> wrist.atSetpoint(),
      wrist
    );

    addCommands(m_shoulderPreset, m_telescopePreset, m_wristPreset);
  }
}

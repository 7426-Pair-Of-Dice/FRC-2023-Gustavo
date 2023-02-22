// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPreset extends SequentialCommandGroup {

  private static ConditionalCommand m_shoulderPreset;
  private static ConditionalCommand m_telescopePreset;
  private static ConditionalCommand m_wristPreset;

  /** Creates a new ArmPreset. */
  public ArmPreset(Shoulder shoulder, Telescope telescope, Wrist wrist, double shoulderAngle, double telescopeDistance, double wristAngle) {
    
    m_shoulderPreset = new RunCommand(() -> shoulder.setPosition(shoulderAngle), shoulder).unless(shoulder::atSetpoint);

    m_telescopePreset = new RunCommand(() -> telescope.setPosition(telescopeDistance), telescope).unless(telescope::atSetpoint);

    m_wristPreset = new RunCommand(() -> wrist.setPosition(wristAngle), wrist).unless(wrist::atSetpoint);

    addCommands(m_shoulderPreset, m_telescopePreset, m_wristPreset);
  }
}
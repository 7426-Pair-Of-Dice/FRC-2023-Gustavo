// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private static Drivetrain m_driveTrain;

  private static DriveTeleop m_driveTeleop;
  private static StartEndCommand m_breakRobot;
  private static AutoBalance m_autoBalance;

  private static CommandXboxController m_xboxController;
  private static Trigger m_aButton;
  private static Trigger m_bButton;

  public RobotContainer() {
    // Input
    m_xboxController = new CommandXboxController(0);
    m_aButton = m_xboxController.a();
    m_bButton = m_xboxController.b();

    // Subsystems
    m_driveTrain = new Drivetrain();

    // Commands
    m_driveTeleop = new DriveTeleop(m_driveTrain, m_xboxController);

    m_breakRobot = new StartEndCommand(() -> m_driveTrain.enableBreak(), () -> m_driveTrain.disableBreak(), m_driveTrain);

    m_autoBalance = new AutoBalance(m_driveTrain);

    m_driveTrain.setDefaultCommand(m_driveTeleop);

    // Configures controller and joystick bindings
    configureBindings();
  }

  private void configureBindings() {
    m_aButton.whileTrue(m_autoBalance);
    m_bButton.whileTrue(m_breakRobot);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

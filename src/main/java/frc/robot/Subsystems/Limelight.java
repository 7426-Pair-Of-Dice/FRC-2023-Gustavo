// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  NetworkTable m_table;
  NetworkTableEntry m_tv;
  NetworkTableEntry m_tx;
  NetworkTableEntry m_ty;
  NetworkTableEntry m_ta;

  double m_xOffset;
  double m_yOffset;
  double m_area;

  /** Creates a new Limelight. */
  public Limelight() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tv = m_table.getEntry("tv");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_xOffset = m_tx.getDouble(0.0);
    m_yOffset = m_ty.getDouble(0.0);
    m_area = m_ta.getDouble(0.0);

    if (this.getCurrentCommand() == null) {
      setPipeline(Constants.Limelight.kCameraPipeline);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Limelight");
    builder.addDoubleProperty("XOffset", this::getXOffset, null);
    builder.addDoubleProperty("YOffset", this::getYOffset, null);
    builder.addDoubleProperty("Area", this::getArea, null);
  }

  public double getXOffset() {
    return m_xOffset;
  }

  public double getYOffset() {
    return m_yOffset;
  }

  public double getArea() {
    return m_area;
  }

  public boolean getTarget() {
    return m_tv.getDouble(0.0) == 1;
  }

  public void setPipeline(int pipeline) {
    m_table.getEntry("pipeline").setNumber(pipeline);
  }
}

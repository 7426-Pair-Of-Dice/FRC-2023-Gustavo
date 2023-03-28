// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private static AddressableLED m_bottomLED;
  private static AddressableLED m_topLeftLED;
  private static AddressableLED m_topRightLED;

  private static AddressableLEDBuffer m_bottomLEDBuffer;
  private static AddressableLEDBuffer m_topLeftLEDBuffer;
  private static AddressableLEDBuffer m_topRightLEDBuffer;

  private static Color m_red;
  private static Color m_gold;
  private static Color m_purple;
  private static Color m_yellow;
  private static Color m_white;
  private static Color m_off;

  private static Timer m_timer;

  private static double m_miliseconds;

  /** Creates a new LED. */
  public LED() {
    m_bottomLED = new AddressableLED(2);
    //m_topLeftLED = new AddressableLED(1);
    //m_topRightLED = new AddressableLED(0);

    m_bottomLEDBuffer = new AddressableLEDBuffer(43);
    //m_topLeftLEDBuffer = new AddressableLEDBuffer(40);
    //m_topRightLEDBuffer = new AddressableLEDBuffer(40);

    m_red = new Color(240, 33, 0);
    m_gold = new Color(228, 211, 0);
    m_purple = new Color(175, 0, 228);
    m_yellow = new Color(243, 243, 0);
    m_white = new Color(255, 255, 255);
    m_off = new Color(0, 0, 0);

    m_timer = new Timer();

    m_timer.start();

    m_bottomLED.setLength(m_bottomLEDBuffer.getLength());
    //m_topLeftLED.setLength(m_topLeftLEDBuffer.getLength());
    //m_topRightLED.setLength(m_topRightLEDBuffer.getLength());

    m_bottomLED.setData(m_bottomLEDBuffer);
    //m_topLeftLED.setData(m_topLeftLEDBuffer);
    //m_topRightLED.setData(m_topRightLEDBuffer);

    m_bottomLED.start();
    //m_topLeftLED.start();
    //m_topRightLED.start();

    for (int i = 0; i < 43; i++) {
      if (i % 2 == 0) {
        m_bottomLEDBuffer.setLED(i, m_yellow);
      } else {
        m_bottomLEDBuffer.setLED(i, m_red);
      }
    }
  }

  @Override
  public void periodic() {
    m_miliseconds = Units.secondsToMilliseconds(Math.floor(m_timer.get()));
    System.out.println(m_miliseconds);
    System.out.println(m_miliseconds);
    if (m_miliseconds % 1000 == 0.0) {
      for (int i = 0; i < 43; i++) {
        if (i % 2 == 0) {
          m_bottomLEDBuffer.setLED(i, m_yellow);
        } else {
          m_bottomLEDBuffer.setLED(i, m_red);
        }
      }
    } else if (m_miliseconds % 500 == 0.0) {
      for (int i = 0; i < 43; i++) {
        if (i % 2 == 0) {
          m_bottomLEDBuffer.setLED(i, m_red);
        } else {
          m_bottomLEDBuffer.setLED(i, m_yellow);
        }
      }
    }

    m_bottomLED.setData(m_bottomLEDBuffer);
  }
}

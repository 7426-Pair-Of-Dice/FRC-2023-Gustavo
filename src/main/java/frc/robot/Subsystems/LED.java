// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  public static enum TopLEDState {
    CUBE_WANTED,
    CONE_WANTED,
    IDLE,
  }

  public static enum BottomLEDState {
    STROBE,
    BLINK_RED
  }

  private static AddressableLED m_led;

  private static AddressableLEDBuffer m_ledBuffer;

  private static Color m_red;
  private static Color m_gold;
  private static Color m_purple;
  private static Color m_yellow;
  private static Color m_white;
  private static Color m_off;

  private static Timer m_timer;

  private static double m_miliseconds;

  private static TopLEDState m_topLEDState;
  private static BottomLEDState m_bottomLEDState;

  /** Creates a new LED. */
  public LED() {
    m_led = new AddressableLED(Constants.LED.kLEDStripID);

    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.kStripLength);

    m_red = new Color(240, 33, 0);
    m_gold = new Color(228, 211, 0);
    m_purple = new Color(175, 0, 228);
    m_yellow = new Color(243, 243, 0);
    m_white = new Color(255, 255, 255);
    m_off = new Color(0, 0, 0);

    m_timer = new Timer();

    m_timer.start();

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);

    m_led.start();

    setTopLEDState(TopLEDState.IDLE);
    setBottomLEDState(BottomLEDState.STROBE);
  }

  @Override
  public void periodic() {
    m_miliseconds = Math.floor(Units.secondsToMilliseconds(m_timer.get()) / 100) * 100;
    
    if (m_bottomLEDState == BottomLEDState.STROBE) {
      strobe();
    } else if (m_bottomLEDState == BottomLEDState.BLINK_RED) {
      blinkRed();
    }

    if (m_topLEDState == TopLEDState.CONE_WANTED) {
      coneWanted();
    } else if (m_topLEDState == TopLEDState.CUBE_WANTED) {
      cubeWanted();
    } else if (m_topLEDState == TopLEDState.IDLE) {
      idle();
    }

    m_led.setData(m_ledBuffer);
  }

  public void setTopLEDState(TopLEDState state) {
    m_topLEDState = state;
  }

  public void setBottomLEDState(BottomLEDState state) {
    m_bottomLEDState = state;
  }

  private void setTopLEDs(Color color) {
    for (int i = 43; i < 123; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  private void setBottomLEDs(Color color) {
    for (int i = 0; i < 44; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  private void strobe() {
    if (m_miliseconds % 1000 == 0.0) {
      for (int i = 0; i < 43; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_gold);
        } else {
          m_ledBuffer.setLED(i, m_red);
        }
      }
    } else if (m_miliseconds % 500 == 0.0) {
      for (int i = 0; i < 43; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_red);
        } else {
          m_ledBuffer.setLED(i, m_gold);
        }
      }
    }

    m_led.setData(m_ledBuffer);
  }

  private void blinkRed() {
    if (m_miliseconds % 1000 == 0.0) {
      setBottomLEDs(m_red);
    } else if (m_miliseconds % 500 == 0.0) {
      setBottomLEDs(m_off);
    }
  }

  private void cubeWanted() {
    setTopLEDs(m_purple);
  }

  private void coneWanted() {
    setTopLEDs(m_yellow);
  }

  private void idle() {
    setTopLEDs(m_white);
  }
}

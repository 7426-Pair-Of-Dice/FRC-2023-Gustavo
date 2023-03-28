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

  public static enum TopLEDState {
    CONE_WANTED,
    CUBE_WANTED,
    IDLE,
  }

  public static enum BottomLEDState {
    STROBE,
    BLINK,
  }

  private static TopLEDState m_topLEDState;
  private static BottomLEDState m_bottomLEDState;

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

  private static ArrayList<Color> m_prevColors;

  private static Timer m_timer;

  private static double m_miliseconds;

  /** Creates a new LED. */
  public LED() {
    m_bottomLED = new AddressableLED(Constants.LED.kBottomLEDId);
    m_topLeftLED = new AddressableLED(Constants.LED.kTopLeftLEDID);
    m_topRightLED = new AddressableLED(Constants.LED.kTopRightLEDID);

    m_bottomLEDBuffer = new AddressableLEDBuffer(Constants.LED.kBottomLEDLength);
    m_topLeftLEDBuffer = new AddressableLEDBuffer(Constants.LED.kTopLeftLEDLength);
    m_topRightLEDBuffer = new AddressableLEDBuffer(Constants.LED.kTopRightLEDLength);

    m_red = new Color(228, 33, 0);
    m_gold = new Color(228, 211, 0);
    m_purple = new Color(175, 0, 228);
    m_yellow = new Color(243, 243, 0);
    m_white = new Color(255, 255, 255);
    m_off = new Color(0, 0, 0);

    m_prevColors = new ArrayList<Color>();

    m_timer = new Timer();

    m_miliseconds = 0.0;

    m_bottomLED.setLength(m_bottomLEDBuffer.getLength());
    m_topLeftLED.setLength(m_topLeftLEDBuffer.getLength());
    m_topRightLED.setLength(m_topRightLEDBuffer.getLength());

    m_bottomLED.setData(m_bottomLEDBuffer);
    m_topLeftLED.setData(m_topLeftLEDBuffer);
    m_topRightLED.setData(m_topRightLEDBuffer);

    m_bottomLED.start();
    m_topLeftLED.start();
    m_topRightLED.start();

    m_timer.start();

    setTopLEDState(TopLEDState.IDLE);
    setBottomLEDState(BottomLEDState.STROBE);

    topSetup();
    bottomSetup();
  }

  @Override
  public void periodic() {

    m_miliseconds = Units.secondsToMilliseconds(Math.floor(m_timer.get()));

    if (m_topLEDState == TopLEDState.CUBE_WANTED) {
      cube();
    } else if (m_topLEDState == TopLEDState.CONE_WANTED) {
      cone();
    } else if (m_topLEDState == TopLEDState.IDLE) {
      idle();
    }

    if (m_bottomLEDState == BottomLEDState.STROBE) {
      strobe();
    } else if (m_bottomLEDState == BottomLEDState.BLINK) {
      blink();
    }
  }
 
  private void topSetup() {
    setTopLEDs(m_white);
  }

  private void bottomSetup() {
    for (int i = 0; i < m_bottomLEDBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_bottomLEDBuffer.setLED(i, m_red);
      } else {
        m_bottomLEDBuffer.setLED(i, m_gold);
      }
      m_prevColors.set(i, m_bottomLEDBuffer.getLED(i));
    }
  }


  public void setTopLEDState(TopLEDState state) {
    m_topLEDState = state;
  }

  public void setBottomLEDState(BottomLEDState state) {
    m_bottomLEDState = state;
  }

  private void setTopLEDs(Color color) {
    for (int i = 0; i < m_topLeftLEDBuffer.getLength(); i++) {
      m_topLeftLEDBuffer.setLED(i, color);
    }

    for (int i = 0; i < m_topRightLEDBuffer.getLength(); i++) {
      m_topRightLEDBuffer.setLED(i, color);
    }
  }

  private void setBottomLEDs(Color color) {
    for (int i = 0; i < m_bottomLEDBuffer.getLength(); i++) {
      m_bottomLEDBuffer.setLED(i, color);
    }
  }

  private void strobe() {
    if (m_miliseconds % 300 == 0.0) {

      for (int i = 0; i < m_bottomLEDBuffer.getLength(); i++) {
        if (m_prevColors.get(i) == m_yellow) {
          m_bottomLEDBuffer.setLED(i, m_red);
        } else {
          m_bottomLEDBuffer.setLED(i, m_gold);
        }
      }

    } else if (m_miliseconds % 150 == 0.0) {
      setBottomLEDs(m_red);
    }
  }

  private void blink() {
    if (m_miliseconds % 300 == 0.0) {
      setTopLEDs(m_red);
    } else if (m_miliseconds % 150 == 0.0) {
      setTopLEDs(m_off);
    }
  }

  private void cube() {
    setTopLEDs(m_purple);
  }

  private void cone() {
    setTopLEDs(m_yellow);
  }

  private void idle() {
    setTopLEDs(m_white);
  }
}

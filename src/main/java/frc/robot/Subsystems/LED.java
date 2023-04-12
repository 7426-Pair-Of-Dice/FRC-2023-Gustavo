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
    IDLE,
    CUBE_WANTED,
    CUBE_HAVE,
    CONE_WANTED,
    CONE_HAVE,
  }

  public static enum BottomLEDState {
    IDLE,
    MINUTE_LEFT,
    THIRTY_SECONDS_LEFT,
    FIFTEEN_SECONDS_LEFT
  }

  private static AddressableLED m_led;

  private static AddressableLEDBuffer m_ledBuffer;

  private static Color m_red;
  private static Color m_gold;
  private static Color m_purple;
  private static Color m_blue;
  private static Color m_yellow;
  private static Color m_white;
  private static Color m_off;

  private static Timer m_timer;

  private static double m_miliseconds;

  private static TopLEDState m_topLEDState;
  private static BottomLEDState m_bottomLEDState;

  private static int m_bottomSnakeIndex;
  private static int m_snakeLength;

  /** Creates a new LED. */
  public LED() {
    m_led = new AddressableLED(Constants.LED.kLEDStripID);

    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.kStripLength);

    m_red = new Color(255, 0, 0);
    m_gold = new Color(240, 205, 0);
    m_blue = new Color(0, 0, 255);
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
    setBottomLEDState(BottomLEDState.IDLE);

    m_bottomSnakeIndex = 0;
    m_snakeLength = 10;
  }

  @Override
  public void periodic() {
    m_miliseconds = Math.floor(Units.secondsToMilliseconds(m_timer.get()) / 100) * 100;

    if (m_bottomLEDState == BottomLEDState.IDLE && m_topLEDState == TopLEDState.IDLE) {
      idleBoth();
      m_led.setData(m_ledBuffer);
      return;
    }
    
    if (m_bottomLEDState == BottomLEDState.IDLE) {
      strobe();
    } else if (m_bottomLEDState == BottomLEDState.MINUTE_LEFT) {
      oneMinuteLeft();
    } else if (m_bottomLEDState == BottomLEDState.THIRTY_SECONDS_LEFT) {
      thirtySecondsLeft();
    } else if (m_bottomLEDState == BottomLEDState.FIFTEEN_SECONDS_LEFT) {
      fifteenSecondsLeft();
    } else if (m_bottomLEDState == BottomLEDState.IDLE) {
      idleBottom();
    }

    if (m_topLEDState == TopLEDState.CONE_HAVE) {
      coneHave();
    } else if (m_topLEDState == TopLEDState.CONE_WANTED) {
      coneWanted();
    } else if (m_topLEDState == TopLEDState.CUBE_WANTED) {
      cubeWanted();
    } else if (m_topLEDState == TopLEDState.CUBE_HAVE) {
      cubeHave();
    } else if (m_topLEDState == TopLEDState.IDLE) {
      idleTop();
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
  }

  private void snake() {
    for (int i = 0; i < 123; i++) {

      // if (m_bottomSnakeIndex - i <= m_snakeLength) {

      // }

      if(((i < m_bottomSnakeIndex) && (i > m_bottomSnakeIndex - Math.max(m_snakeLength, 0))) || (i > Math.min(123, 123 - m_snakeLength + m_bottomSnakeIndex))) {
        // if (i % 2 == 0) {
        //   m_ledBuffer.setLED(i, m_yellow);
        // } else {
        //   m_ledBuffer.setLED(i, m_red);
        // }
        m_ledBuffer.setHSV(i, ((i * 120) / 124) % 180, 255, 128);
      } else {
        m_ledBuffer.setLED(i, m_off);
      }
    }
    
    if (m_bottomSnakeIndex > 123) {
      m_bottomSnakeIndex = 0;
    }

    m_bottomSnakeIndex++;
  }

  private void idleTop() {
    if (m_miliseconds % 1000 == 0.0) {
      for (int i = 43; i < 123; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_gold);
        } else {
          m_ledBuffer.setLED(i, m_red);
        }
      }
    } else if (m_miliseconds % 500 == 0.0) {
      for (int i = 43; i < 123; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_red);
        } else {
          m_ledBuffer.setLED(i, m_gold);
        }
      }
    }
  }

  private void coneWanted() {
    if (m_miliseconds % 1000 == 0.0) {
      setTopLEDs(m_yellow);
    } else if (m_miliseconds % 500 == 0.0) {
      setTopLEDs(m_off);
    }
  }

  private void coneHave() {
    setTopLEDs(m_yellow);
  }

  private void cubeWanted() {
    if (m_miliseconds % 1000 == 0.0) {
      setTopLEDs(m_purple);
    } else if (m_miliseconds % 500 == 0.0) {
      setTopLEDs(m_off);
    }
  }

  private void cubeHave() {
    setTopLEDs(m_purple);
  }

  private void idleBottom() {
    strobe();
  }

  private void oneMinuteLeft() {
    if (m_miliseconds % 1000 == 0.0) {
      for (int i = 0; i < 43; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_red);
        } else {
          m_ledBuffer.setLED(i, m_off);
        }
      }
    } else if (m_miliseconds % 500 == 0.0) {
      for (int i = 0; i < 43; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_off);
        } else {
          m_ledBuffer.setLED(i, m_red);
        }
      }
    }
  }

  private void thirtySecondsLeft() {
    setBottomLEDs(m_red);
  }

  private void fifteenSecondsLeft() {
    if (m_miliseconds % 1000 == 0.0) {
      setBottomLEDs(m_red);
    } else if (m_miliseconds % 500 == 0.0) {
      setBottomLEDs(m_off);
    }
  }

  private void idleBoth() {
    snake();
    /* if (m_miliseconds % 1000 == 0.0) {
      for (int i = 0; i < 123; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_gold);
        } else {
          m_ledBuffer.setLED(i, m_red);
        }
      }
    } else if (m_miliseconds % 500 == 0.0) {
      for (int i = 0; i < 123; i++) {
        if (i % 2 == 0) {
          m_ledBuffer.setLED(i, m_red);
        } else {
          m_ledBuffer.setLED(i, m_gold);
        }
      }
    } */
  }
}

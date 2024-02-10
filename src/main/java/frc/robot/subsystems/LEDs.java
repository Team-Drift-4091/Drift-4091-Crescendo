// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utility.LEDString;
import frc.robot.utility.LEDString.AlternatingPattern;
import frc.robot.utility.LEDString.LEDPattern;
import frc.robot.utility.LEDString.LEDPatterns;

@SuppressWarnings("unused")
public class LEDs extends SubsystemBase {
  private static LEDs instance = null;
  public static LEDs getInstance() {
    if (instance == null) {
      instance = new LEDs();
    }
    return instance;
  }

  public static final int LENGTH = 147;
  public enum LEDMode {
    OFF,
    DISCONNECTED,
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TELEOP_SPECIAL,
    FAULT,
    CUBE_TARGET,
    CUBE_HOLD;
  }
  private LEDMode currentMode = null; // Set to OFF on init

  private final LEDString leds;

  private final AtomicReference<LEDPattern> ledPattern = new AtomicReference<LEDPattern>(null);
  private final Notifier periodicThread;

  private LEDs() {
    leds = new LEDString(9, LENGTH);

    periodicThread = new Notifier(() -> {
      LEDPattern pattern = ledPattern.get();
      if (pattern != null) {
        pattern.run(leds);
      }
    });
    periodicThread.setName("LED periodic");
    periodicThread.startPeriodic(.02);

    setMode(LEDMode.OFF);
  }

  public void start() {
    leds.start();
  }

  public void stop() {
    leds.stop();
  }

  public void setCustomPattern(LEDPattern customPattern, boolean nonLooping) {
    if (nonLooping) {
      ledPattern.set(new NonLoopingPattern(customPattern));
    } else {
      ledPattern.set(customPattern);
    }
    currentMode = null;
  }

  public void setCustomPattern(LEDPattern customPattern) {
    ledPattern.set(customPattern);
    currentMode = null;
  }

  public void setMode(LEDMode mode) {
    if (mode != currentMode) {
      currentMode = mode;
      switch (currentMode) {
        case OFF:
          ledPattern.set(LEDPatterns.OFF);
          break;
        case DISCONNECTED:
          // ledPattern.set(LEDs.DISCONNECTED);
          ledPattern.set(LOADING);
          break;
        case DISABLED:
          ledPattern.set(new NonLoopingPattern(LEDPattern.setRGB(0, 50, 0)));
          break;
        case AUTONOMOUS:
          ledPattern.set(LEDPatterns.RAINBOW);
          break;
        case TELEOP:
          // Show alliance color
    
          break;
        case TELEOP_SPECIAL:
          // Show alliance color
      }
    }
  }

  private void off(LEDString leds) {
    leds.off();
    leds.update();
    ledPattern.set(null);
  }

  private void twoColor(LEDString leds, int spacing, int r1, int g1, int b1, int r2, int g2, int b2) {
    for (int i = 0; i < LENGTH; i++) {
      if (i / spacing % 2 == 0) {
        leds.setRGB(i, r1, g1, b1);
      } else {
        leds.setRGB(i, r2, g2, b2);
      }
    }
  }

  @Override
  public void periodic() {}

  public class NonLoopingPattern implements LEDPattern {
    private final LEDPattern pattern;
    public NonLoopingPattern(LEDPattern pattern) {
      this.pattern = pattern;
    }

    @Override
    public void run(LEDString leds) {
      pattern.run(leds);
      ledPattern.set(null);
    }

    @Override
    public void draw(LEDString leds) {
      pattern.draw(leds);
    }
  }

  // loading animation used at comp
  public static final LEDPattern DISCONNECTED = new LEDPattern() {
    private int i = 0;
    private boolean isIncreasing = true;
    public void draw(LEDString leds) {
      if (i + 4 > LENGTH) {
        isIncreasing = false;
      }
      if (i == 0) {
        isIncreasing = true;
      }
      i += isIncreasing ? 1 : -1;
      leds.setRGB((i) % LENGTH, 150, 150, 150, true);
      leds.setRGB((i + 1) % LENGTH, 150, 150, 150);
      leds.setRGB((i + 2) % LENGTH, 150, 150, 150);
      leds.setRGB((i + 3) % LENGTH, 150, 150, 150);
      leds.update();
    }
  };

  public static final LEDPattern LOADING = new LEDPattern() {
    private int targetIndex = 0;
    private int currentIndex = 0;
    private boolean increasing = true;

    private int length = 1;

    private boolean inverted = false;
    @Override
    public void draw(LEDString leds) {        
      if (currentIndex == targetIndex) {
        if (increasing) {
          targetIndex = (int) (Math.random() * (currentIndex - length));
          currentIndex = currentIndex - length;
        } else {
          targetIndex = (int) ((currentIndex + length) + Math.random() * (leds.length - (currentIndex +  length)));
          currentIndex = currentIndex + length;
        }
        increasing = !increasing;
        length++;

      }
      currentIndex += Math.signum(targetIndex - currentIndex);

      if (inverted) {
        leds.setAllRGB(20, 150, 0);
        leds.setRGB(targetIndex, 0, 0, 0, false);
        for (int i = 0; i < length; i++) {
          if (increasing && currentIndex - i > -1) {
            leds.setRGB(currentIndex - i, 100, 100, 100, false); 
          } else if (currentIndex + i < leds.length) {
            leds.setRGB(currentIndex + i, 100, 100, 100, false); 
          }
        }
      } else {
        leds.setAllRGB(100, 100, 100);
        leds.setRGB(targetIndex, 0, 0, 0, false);
        for (int i = 0; i < length; i++) {
          if (increasing && currentIndex - i > -1) {
            leds.setRGB(currentIndex - i, 20, 150, 0, false); 
          } else if (currentIndex + i < leds.length) {
            leds.setRGB(currentIndex + i, 20, 150, 0, false); 
          }
        }
      }
      if (length == leds.length) {
        inverted = !inverted;
        length = 1;
        increasing = true;
        currentIndex = 0;
        targetIndex = 0;
      }
    }
  };
}

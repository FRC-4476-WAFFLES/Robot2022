// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightController extends SubsystemBase {
  private final PWMSparkMax blinkin = new PWMSparkMax(Constants.lightsBlinkin);

  private final Timer blinkTimer = new Timer();
  private double colour1 = LightColours.YELLOW.getPWMSignal();
  private double colour2 = LightColours.YELLOW.getPWMSignal();
  private boolean isBlinkColour1 = true;

  private final double blinkRate = 0.1;

  public enum LightColours {
    PINK(0.57),
    RED(0.61),
    GREEN(0.77),
    BLUE(0.87),
    YELLOW(0.69),
    BLACK(0.99);

    private final double pwmSignal;

    LightColours(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }

    public double getPWMSignal() {
      return pwmSignal;
    }
  }

  /** Creates a new LightController. */
  public LightController() {
    blinkTimer.reset();
    blinkTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (blinkTimer.get() > blinkRate) {
      blinkin.set(isBlinkColour1 ? colour1 : colour2);
      isBlinkColour1 = isBlinkColour1 ? false : true;
      blinkTimer.reset();
    }
  }

  public void setRawLightColour(double pwmSignal) {
    this.colour1 = pwmSignal;
    this.colour2 = pwmSignal;
  }

  public void setLightColour(LightColours colour) {
    this.colour1 = colour.getPWMSignal();
    this.colour2 = colour.getPWMSignal();
  }

  public void blinkBetweenColours(LightColours colour1, LightColours colour2) {
    this.colour1 = colour1.getPWMSignal();
    this.colour2 = colour2.getPWMSignal();
  }
}

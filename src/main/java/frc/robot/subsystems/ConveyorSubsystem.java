// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private final CANSparkMax conveyorSpin = new CANSparkMax(Constants.conveyorSpin, MotorType.kBrushless);
  private final DigitalInput lowIR = new DigitalInput(Constants.lowIR);
  private final DigitalInput midIR = new DigitalInput(Constants.midIR);
  private final DigitalInput highIR = new DigitalInput(Constants.highIR);

  private boolean lowIRPreviousState = false;
  
  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    conveyorSpin.restoreFactoryDefaults();
    conveyorSpin.setSmartCurrentLimit(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("High IR", getHighIR());
    SmartDashboard.putBoolean("Mid IR", getMidIR());
    SmartDashboard.putBoolean("Low IR", getLowIR());
    SmartDashboard.putBoolean("Low IR Stored State", lowIRPreviousState);

    if (getLowIR()) {
      lowIRPreviousState = true;
    }

    if (!getLowIR() && getMidIR()) {
      lowIRPreviousState = false;
    }
  }

  public void runConveyorIn() {
    conveyorSpin.set(.4);
  }

  public void runConveyorOut() {
    conveyorSpin.set(-1);
  }

  public void runConveyor(double power) {
    conveyorSpin.set(power);
  }

  public void stopConveyor() {
    conveyorSpin.set(0);
  }

  public boolean getHighIR() {
    return !highIR.get();
  }

  public boolean getMidIR() {
    return !midIR.get();
  }

  public boolean getLowIR() {
    return !lowIR.get();
  }

  public boolean getLowIRPreviousState() {
    return lowIRPreviousState;
  }

  public boolean shouldRun() {
    if (getHighIR()) {
      return false;
    }

    if (getMidIR() && (getLowIR() || lowIRPreviousState)) {
      return true;
    }

    if (getMidIR()) {
      return false;
    }

    if (getLowIR() || lowIRPreviousState) {
      return true;
    }

    return false;
  }
}

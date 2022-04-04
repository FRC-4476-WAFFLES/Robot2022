// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private final CANSparkMax conveyorSpin = new CANSparkMax(Constants.conveyorSpin, MotorType.kBrushless);
  private final SparkMaxPIDController pidController = conveyorSpin.getPIDController();

  private final DigitalInput lowIR = new DigitalInput(Constants.lowIR);
  private final DigitalInput midIR = new DigitalInput(Constants.midIR);
  private final DigitalInput highIR = new DigitalInput(Constants.highIR);

  private final double maxRPM = 11000;

  private boolean lowIRPreviousState = false;
  
  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    conveyorSpin.restoreFactoryDefaults();
    conveyorSpin.setSmartCurrentLimit(30);

    pidController.setP(0);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0.001);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("High IR", getHighIR());
    SmartDashboard.putBoolean("Mid IR", getMidIR());
    SmartDashboard.putBoolean("Low IR", getLowIR());
    SmartDashboard.putBoolean("Low IR Stored State", lowIRPreviousState);

    SmartDashboard.putNumber("Conveyor Encoder Speed", conveyorSpin.getEncoder().getVelocity());

    if (getLowIR()) {
      lowIRPreviousState = true;
    }

    if (!getLowIR() && getMidIR()) {
      lowIRPreviousState = false;
    }
  }

  public void runConveyor(double speed) {
    SmartDashboard.putNumber("Conveyor Power", speed);
    SmartDashboard.putNumber("Conveyor Target Speed", speed * maxRPM);
    //System.err.println("Conveyor is being told to run at power " + String.valueOf(power));
    conveyorSpin.set(speed);
    //pidController.setReference(speed * maxRPM, ControlType.kVelocity);
  }

  public void stopConveyor() {
    //System.err.println("Conveyor is being told to stop");
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

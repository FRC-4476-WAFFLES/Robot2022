// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private final TalonSRX conveyorSpin = new TalonSRX(Constants.conveyorSpin);
  
  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    conveyorSpin.configFactoryDefault();
    conveyorSpin.configContinuousCurrentLimit(30);
    conveyorSpin.configPeakCurrentLimit(30);
    conveyorSpin.enableCurrentLimit(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runConveyorIn() {
    conveyorSpin.set(ControlMode.PercentOutput, 1);
  }

  public void runConveyorOut() {
    conveyorSpin.set(ControlMode.PercentOutput, -1);
  }

  public void stopConveyor() {
    conveyorSpin.set(ControlMode.PercentOutput, 0);
  }
}

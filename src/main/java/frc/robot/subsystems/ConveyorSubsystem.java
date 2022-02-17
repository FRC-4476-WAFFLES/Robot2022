// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private final CANSparkMax conveyorSpin = new CANSparkMax(Constants.conveyorSpin, MotorType.kBrushless);
  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    conveyorSpin.restoreFactoryDefaults();
    conveyorSpin.setSmartCurrentLimit(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runConveyorIn() {
    conveyorSpin.set(1);
  }

  public void runConveyorOut() {
    conveyorSpin.set(-1);
  }

  public void stopConveyor() {
    conveyorSpin.set(0);
  }
}

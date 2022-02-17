// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeSpin = new TalonSRX(Constants.intakeSpin);
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeSpin.configFactoryDefault();
    intakeSpin.configContinuousCurrentLimit(40);
    intakeSpin.configPeakCurrentLimit(40);
    intakeSpin.enableCurrentLimit(true);
    intakeSpin.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntakeIn() {
    intakeSpin.set(ControlMode.PercentOutput, 1);
  }

  public void runIntakeOut() {
    intakeSpin.set(ControlMode.PercentOutput, -1);
  }

  public void stopIntake() {
    intakeSpin.set(ControlMode.PercentOutput, 0);
  }
}
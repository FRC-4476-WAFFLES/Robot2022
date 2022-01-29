// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeSpin = new TalonSRX(Constants.intakeSpin);
  private final CANSparkMax deployLeader = new CANSparkMax(Constants.intakeDeploy1, MotorType.kBrushless);
  private final CANSparkMax deployFollower = new CANSparkMax(Constants.intakeDeploy2, MotorType.kBrushless);

  private final DigitalInput limit = new DigitalInput(Constants.intakeLimit);

  private final RelativeEncoder encoder = deployLeader.getEncoder();
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeSpin.configFactoryDefault();
    intakeSpin.configContinuousCurrentLimit(30);
    intakeSpin.configPeakCurrentLimit(30);
    intakeSpin.enableCurrentLimit(true);

    deployLeader.setSmartCurrentLimit(30);
    deployLeader.setIdleMode(IdleMode.kBrake);

    deployFollower.setSmartCurrentLimit(30);
    deployFollower.setIdleMode(IdleMode.kBrake);
    deployFollower.follow(deployLeader, true);
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

  public void deployIntake() {
    if (!limit.get()) {
      deployLeader.getEncoder(Type.kHallSensor, 5);
    }
  }
}

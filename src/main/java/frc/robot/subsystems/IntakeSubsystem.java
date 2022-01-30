// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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
  private final DigitalInput lowIR = new DigitalInput(Constants.lowIR);
  private final DigitalInput midIR = new DigitalInput(Constants.midIR);
  private final DigitalInput highIR = new DigitalInput(Constants.highIR);

  private final RelativeEncoder deployEncoder = deployLeader.getEncoder(Type.kHallSensor, 42);
  private final SparkMaxPIDController deployPIDController = deployLeader.getPIDController();
  private final double deployTarget = 21.25; // The approximate number of rotations of the deploy motors to deploy the intake
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeSpin.configFactoryDefault();
    intakeSpin.configContinuousCurrentLimit(30);
    intakeSpin.configPeakCurrentLimit(30);
    intakeSpin.enableCurrentLimit(true);

    deployLeader.restoreFactoryDefaults();
    deployLeader.setSmartCurrentLimit(20);
    deployLeader.setIdleMode(IdleMode.kBrake);

    deployFollower.restoreFactoryDefaults();
    deployFollower.setSmartCurrentLimit(20);
    deployFollower.setIdleMode(IdleMode.kBrake);
    deployFollower.follow(deployLeader, true);

    // PID coefficients
    // TODO: set these values to better values determined through testing. 
    kP = 0.1; 
    kI = 0;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    deployPIDController.setP(kP);
    deployPIDController.setI(kI);
    deployPIDController.setD(kD);
    deployPIDController.setIZone(kIz);
    deployPIDController.setFF(kFF);
    deployPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!limit.get()) {
      deployEncoder.setPosition(0);
    }
  }

  public boolean HighIR() {
    return !highIR.get();
  }

  public boolean LowIR() {
    return !lowIR.get();
  }

  public boolean MidIR() {
    return !midIR.get();
  }

  public boolean shouldRun() {
    return !(HighIR() && LowIR()) || !(HighIR() && MidIR()); 
  }

  public boolean shouldConveyorRun() {
    return !(HighIR() && MidIR());
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
    deployPIDController.setReference(deployTarget, ControlType.kPosition);
  }

  public void retractIntake() {
    deployPIDController.setReference(0, ControlType.kPosition);
  }
}

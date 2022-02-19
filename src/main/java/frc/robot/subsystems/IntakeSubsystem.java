// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeSpin = new TalonSRX(Constants.intakeSpin);
  private final CANSparkMax deployLeft = new CANSparkMax(Constants.intakeDeployLeft, MotorType.kBrushless);
  private final CANSparkMax deployRight = new CANSparkMax(Constants.intakeDeployRight, MotorType.kBrushless);
  
  private final DigitalInput lowIR = new DigitalInput(Constants.lowIR);
  private final DigitalInput midIR = new DigitalInput(Constants.midIR);
  private final DigitalInput highIR = new DigitalInput(Constants.highIR);

  private final RelativeEncoder encoderLeft = deployLeft.getEncoder(Type.kHallSensor, 42);
  private final RelativeEncoder encoderRight = deployRight.getEncoder(Type.kHallSensor, 42);
  private final SparkMaxPIDController leftPIDController = deployLeft.getPIDController();
  //private final SparkMaxPIDController rightPIDController = deployRight.getPIDController();
  private final double deployOvershootTarget = -50;
  private double deployTargetLeft = 0;
  private double deployTargetRight = 0;
  //private final double deployOvershootTarget = -21.14; // The approximate number of rotations of the deploy motors to deploy the intake
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeSpin.configFactoryDefault();
    intakeSpin.configContinuousCurrentLimit(40);
    intakeSpin.configPeakCurrentLimit(40);
    intakeSpin.enableCurrentLimit(true);
    intakeSpin.setInverted(true);

    deployLeft.restoreFactoryDefaults();
    deployLeft.setSmartCurrentLimit(20);
    deployLeft.setIdleMode(IdleMode.kBrake);

    deployRight.restoreFactoryDefaults();
    deployRight.setSmartCurrentLimit(20);
    deployRight.setIdleMode(IdleMode.kBrake);
    //deployRight.setInverted(true);
    deployRight.follow(deployLeft, true);

    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);

    // PID coefficients
    // TODO: set these values to better values determined through testing. 
    kP = 0.1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    leftPIDController.setP(kP);
    leftPIDController.setI(kI);
    leftPIDController.setD(kD);
    leftPIDController.setIZone(kIz);
    leftPIDController.setFF(kFF);
    leftPIDController.setOutputRange(kMinOutput, kMaxOutput);
/*
    rightPIDController.setP(kP);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);
    rightPIDController.setIZone(kIz);
    rightPIDController.setFF(kFF);
    rightPIDController.setOutputRange(kMinOutput, kMaxOutput);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (encoderLeft.getPosition() < -5 && encoderLeft.getVelocity() == 0) {
      deployTargetLeft = encoderLeft.getPosition();
    }

    if (encoderRight.getPosition() < -5 && encoderRight.getVelocity() == 0) {
      deployTargetRight = encoderRight.getPosition();
    }

    leftPIDController.setReference(deployTargetLeft, CANSparkMax.ControlType.kPosition);
    //rightPIDController.setReference(deployTargetRight, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putBoolean("High IR", getHighIR());
    SmartDashboard.putBoolean("Mid IR", getMidIR());
    SmartDashboard.putBoolean("Low IR", getLowIR());

    SmartDashboard.putNumber("Left Encoder Location", encoderLeft.getPosition());
    SmartDashboard.putNumber("Right Encoder Location", encoderRight.getPosition());
    SmartDashboard.putNumber("Left Deploy Target", deployTargetLeft);
    SmartDashboard.putNumber("Right Deploy Target", deployTargetRight);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { leftPIDController.setP(p); kP = p; }
    if((i != kI)) { leftPIDController.setI(i); kI = i; }
    if((d != kD)) { leftPIDController.setD(d); kD = d; }
    if((iz != kIz)) { leftPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { leftPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      leftPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
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

  public boolean shouldRun() {
    return !(getHighIR() && getLowIR()) || !(getHighIR() && getMidIR()); 
  }

  public boolean shouldConveyorRun() {
    return !(getHighIR() && getMidIR());
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
    deployTargetLeft = deployOvershootTarget;
    deployTargetRight = deployOvershootTarget;
  }

  public void retractIntake() {
    deployTargetLeft = 0;
    deployTargetRight = 0;
  }

  public void resetEncoder() {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }
}
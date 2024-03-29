// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeSpin = new TalonSRX(Constants.intakeSpin);
  private final CANSparkMax deployLeft = new CANSparkMax(Constants.intakeDeployLeft, MotorType.kBrushless);
  private final CANSparkMax deployRight = new CANSparkMax(Constants.intakeDeployRight, MotorType.kBrushless);

  private final RelativeEncoder encoderLeft = deployLeft.getEncoder(Type.kHallSensor, 42);
  private final RelativeEncoder encoderRight = deployRight.getEncoder(Type.kHallSensor, 42);
  private final SparkMaxPIDController leftPIDController = deployLeft.getPIDController();
  private final SparkMaxPIDController rightPIDController = deployRight.getPIDController();
  //private final double deployOvershootTarget = -50;
  private final double leftDeployedPosition = -22.12;
  private final double rightDeployedPosition = -21.1;
  private double deployTargetLeft = 0;
  private double deployTargetRight = 0;
  //private final double deployOvershootTarget = -21.14; // The approximate number of rotations of the deploy motors to deploy the intake
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeSpin.configFactoryDefault();
    intakeSpin.configContinuousCurrentLimit(15);
    intakeSpin.configPeakCurrentLimit(15);
    intakeSpin.enableCurrentLimit(true);
    intakeSpin.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    intakeSpin.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, 100);
    intakeSpin.setControlFramePeriod(ControlFrame.Control_3_General, 100);
    intakeSpin.setInverted(false);

    deployLeft.restoreFactoryDefaults();
    deployLeft.setSmartCurrentLimit(20);
    deployLeft.setIdleMode(IdleMode.kBrake);
    deployLeft.setControlFramePeriodMs(40);

    deployRight.restoreFactoryDefaults();
    deployRight.setSmartCurrentLimit(20);
    deployRight.setIdleMode(IdleMode.kBrake);
    deployRight.setControlFramePeriodMs(40);
    deployRight.setInverted(true);

    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);

    // PID coefficients
    // TODO: set these values to better values determined through testing. 
    kP = 0.1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.50; 
    kMinOutput = -0.50;

    // set PID coefficients
    leftPIDController.setP(kP);
    leftPIDController.setI(kI);
    leftPIDController.setD(kD);
    leftPIDController.setIZone(kIz);
    leftPIDController.setFF(kFF);
    leftPIDController.setOutputRange(kMinOutput, kMaxOutput);

    rightPIDController.setP(kP);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);
    rightPIDController.setIZone(kIz);
    rightPIDController.setFF(kFF);
    rightPIDController.setOutputRange(kMinOutput, kMaxOutput);
    
    SmartDashboard.setDefaultNumber("Intake kP", kP);
    SmartDashboard.setDefaultNumber("Intake kI", kI);
    SmartDashboard.setDefaultNumber("Intake kD", kD);
    SmartDashboard.setDefaultNumber("Intake kIZone", kIz);
    SmartDashboard.setDefaultNumber("Intake kF", kFF);
    SmartDashboard.setDefaultNumber("Intake kMax", kMaxOutput);
    SmartDashboard.setDefaultNumber("Intake kMin", kMinOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    if (encoderLeft.getPosition() < -5 && encoderLeft.getVelocity() == 0) {
      deployTargetLeft = encoderLeft.getPosition();
    }

    if (encoderRight.getPosition() < -5 && encoderRight.getVelocity() == 0) {
      deployTargetRight = encoderRight.getPosition();
    }*/

    leftPIDController.setReference(deployTargetLeft, CANSparkMax.ControlType.kPosition);
    rightPIDController.setReference(deployTargetRight, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("Intake Left Encoder Location", encoderLeft.getPosition());
    SmartDashboard.putNumber("Intake Right Encoder Location", encoderRight.getPosition());
    SmartDashboard.putNumber("Intake Left Deploy Target", deployTargetLeft);
    SmartDashboard.putNumber("Intake Right Deploy Target", deployTargetRight);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Intake kP", 0);
    double i = SmartDashboard.getNumber("Intake kI", 0);
    double d = SmartDashboard.getNumber("Intake kD", 0);
    double iz = SmartDashboard.getNumber("Intake kIZone", 0);
    double ff = SmartDashboard.getNumber("Intake kF", 0);
    double max = SmartDashboard.getNumber("Intake kMax", 0);
    double min = SmartDashboard.getNumber("Intake kMin", 0);

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

    if((p != kP)) { rightPIDController.setP(p); kP = p; }
    if((i != kI)) { rightPIDController.setI(i); kI = i; }
    if((d != kD)) { rightPIDController.setD(d); kD = d; }
    if((iz != kIz)) { rightPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { rightPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      rightPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  public void runIntake(double power) {
    intakeSpin.set(ControlMode.PercentOutput, power);
  }

  public void stopIntake() {
    intakeSpin.set(ControlMode.PercentOutput, 0);
  }

  public void deployIntake() {
    deployTargetLeft = leftDeployedPosition;
    deployTargetRight = rightDeployedPosition;
  }

  public void retractIntake() {
    deployTargetLeft = 0;
    deployTargetRight = 0;
  }

  public void resetEncoder() {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  public double getLeftPosition() {
    return encoderLeft.getPosition();
  }

  public double getRightPosition() {
    return encoderRight.getPosition();
  }

  public double getLeftTarget() {
    return deployTargetLeft;
  }

  public double getRightTarget() {
    return deployTargetRight;
  }
}
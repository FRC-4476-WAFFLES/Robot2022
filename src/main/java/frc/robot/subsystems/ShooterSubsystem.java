// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeader = new TalonFX(Constants.shooterSpinLeft);
  private final TalonFX shooterFollower = new TalonFX(Constants.shooterSpinRight);
  private final TalonSRX kickerWheel = new TalonSRX(Constants.kickerWheelSpin);
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.shooterAngle, MotorType.kBrushless);

  private final SparkMaxPIDController hoodPIDController = hoodMotor.getPIDController();
  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder(Type.kHallSensor, 42);

  private final LinearFilter shooterRPMFilter = LinearFilter.movingAverage(10);

  private double filteredShooterRPM = 0;

  private double shooterkP = ShooterConstants.shooterkP;
  private double shooterkI = ShooterConstants.shooterkI;
  private double shooterkD = ShooterConstants.shooterkD;
  private double shooterkF = ShooterConstants.shooterkF;

  //private double kP = ShooterConstants.anglekP;
  /*
  Tuned PID values:
  p = 0.4
  i = 0
  d = 0.4
  */
  private double kP = 5;
  private double kI = ShooterConstants.anglekI;
  private double kD = ShooterConstants.anglekD;
  private double kF = ShooterConstants.anglekF;
  private double kIz = 0;
  private double kMaxOutput = 1.0;
  private double kMinOutput = -1.0;

  private double shooterTargetRPM = 0;
  private double kickerTargetSpeed = 0;
  private double targetAngle = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterLeader.configFactoryDefault();
    shooterFollower.configFactoryDefault();

    shooterFollower.follow(shooterLeader);
    shooterFollower.setInverted(InvertType.OpposeMaster);
    shooterFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    shooterFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, 100);

    shooterLeader.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    shooterLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooterLeader.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
    shooterLeader.configVelocityMeasurementWindow(4);
    shooterLeader.config_kP(0, shooterkP);
    shooterLeader.config_kI(0, shooterkI);
    shooterLeader.config_kD(0, shooterkD);
    shooterLeader.config_kF(0, shooterkF);
    shooterLeader.configVoltageCompSaturation(12);
    shooterLeader.enableVoltageCompensation(true);
    shooterLeader.setNeutralMode(NeutralMode.Coast);
    shooterLeader.setInverted(true);

    kickerWheel.configFactoryDefault();
    kickerWheel.configContinuousCurrentLimit(30);
    kickerWheel.configPeakCurrentLimit(30);
    kickerWheel.enableCurrentLimit(true);
    kickerWheel.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    kickerWheel.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, 100);
    kickerWheel.setControlFramePeriod(ControlFrame.Control_3_General, 100);
    kickerWheel.setNeutralMode(NeutralMode.Coast);

    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setSmartCurrentLimit(20);
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setInverted(true);

    hoodEncoder.setPosition(0);

    hoodPIDController.setP(kP);
    hoodPIDController.setI(kI);
    hoodPIDController.setD(kD);
    hoodPIDController.setFF(kF);
    hoodPIDController.setIZone(kIz);
    hoodPIDController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.setDefaultNumber("Set Shooter Target Angle", -1.0);
    SmartDashboard.setDefaultNumber("Set Shooter Hood Motor Position", 0);
    SmartDashboard.setDefaultNumber("Set Shooter Target RPM", 0);
    SmartDashboard.setDefaultNumber("Set Kicker Wheel Target Speed", 0);
    SmartDashboard.setDefaultNumber("Set Shooter Angle Offset", 0);

    SmartDashboard.setDefaultNumber("Shooter kP", shooterkP);
    SmartDashboard.setDefaultNumber("Shooter kI", shooterkI);
    SmartDashboard.setDefaultNumber("Shooter kD", shooterkD);
    SmartDashboard.setDefaultNumber("Shooter kF", shooterkF);
    
    SmartDashboard.setDefaultNumber("Hood kP", kP);
    SmartDashboard.setDefaultNumber("Hood kI", kI);
    SmartDashboard.setDefaultNumber("Hood kD", kD);
    SmartDashboard.setDefaultNumber("Hood kIZone", kIz);
    SmartDashboard.setDefaultNumber("Hood kF", kF);
    SmartDashboard.setDefaultNumber("Hood kMax", kMaxOutput);
    SmartDashboard.setDefaultNumber("Hood kMin", kMinOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Current RPM", ticksPer100msToRPM(shooterLeader.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter Raw Velocity", shooterLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Target RPM", shooterTargetRPM);
    SmartDashboard.putNumber("Shooter Target Angle", targetAngle);
    SmartDashboard.putNumber("Shooter Hood Current Motor Position", hoodEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Hood Applied Output", hoodMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Hood Target Position", targetAngle);

    double shooterP = SmartDashboard.getNumber("Shooter kP", 0);
    double shooterI = SmartDashboard.getNumber("Shooter kI", 0);
    double shooterD = SmartDashboard.getNumber("Shooter kD", 0);
    double shooterF = SmartDashboard.getNumber("Shooter kF", 0);

    // read PID coefficients from SmartDashboard
    
    double p = SmartDashboard.getNumber("Hood kP", 0);
    double i = SmartDashboard.getNumber("Hood kI", 0);
    double d = SmartDashboard.getNumber("Hood kD", 0);
    double iz = SmartDashboard.getNumber("Hood kIZone", 0);
    double ff = SmartDashboard.getNumber("Hood kF", 0);
    double max = SmartDashboard.getNumber("Hood kMax", 0);
    double min = SmartDashboard.getNumber("Hood kMin", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { hoodPIDController.setP(p); kP = p; }
    if((i != kI)) { hoodPIDController.setI(i); kI = i; }
    if((d != kD)) { hoodPIDController.setD(d); kD = d; }
    if((iz != kIz)) { hoodPIDController.setIZone(iz); kIz = iz; }
    if((ff != kF)) { hoodPIDController.setFF(ff); kF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      hoodPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if (shooterP != shooterkP) {
      shooterLeader.config_kP(0, shooterP);
      shooterkP = shooterP;
    }
    if (shooterI != shooterkI) {
      shooterLeader.config_kI(0, shooterI);
      shooterkI = shooterI;
    }
    if (shooterD != shooterkD) {
      shooterLeader.config_kD(0, shooterD);
      shooterkD = shooterD;
    }
    if (shooterF != shooterkF) {
      shooterLeader.config_kF(0, shooterF);
      shooterkF = shooterF;
    }

    filteredShooterRPM = shooterRPMFilter.calculate(getShooterRPM());
  }

  public void setKickerSpeed(double target) {
    //System.err.println("Kicker is being told to spin at power" + String.valueOf(target));
    target = clamp(target, -1, 1);
    kickerWheel.set(ControlMode.PercentOutput, target);
  }

  public void setShooterSpeed(double target) {
    target = clamp(target, -6000, 6000);
    shooterTargetRPM = target;
    shooterLeader.set(ControlMode.Velocity, rpmToTicksPer100ms(target));
  }

  public void setHoodAngle(double target) {
    //targetAngle = target = clamp(target, ShooterConstants.minAngle, ShooterConstants.maxAngle);
    //hoodPIDController.setReference(hoodAngleDegreesToRotations(target), ControlType.kPosition);
    targetAngle = target = clamp(target, 0, 13.5);
    hoodPIDController.setReference(targetAngle, ControlType.kPosition);
  }

  public void stop() {
    //System.err.println("Shooter Is being told to stop");
    shooterLeader.set(ControlMode.PercentOutput, 0);
    kickerWheel.set(ControlMode.PercentOutput, 0);
    hoodMotor.set(0);
  }

  public void driverStationShooterRPM() {
    shooterTargetRPM = SmartDashboard.getNumber("Set Shooter Target RPM", 0);
    setShooterSpeed(shooterTargetRPM);
  }

  public void driverStationAngleControl() {/*
    targetAngle = SmartDashboard.getNumber("Set Shooter Target Angle", 0);
    setHoodAngle(targetAngle);*/
    //System.err.println("Hello there. ");
    targetAngle = SmartDashboard.getNumber("Set Shooter Hood Motor Position", 0);
    setHoodAngle(targetAngle);
    /*
    targetAngle = clamp(targetAngle, 0, 13.5);
    System.err.println(targetAngle);
    hoodPIDController.setReference(targetAngle, ControlType.kPosition);*/
  }

  public void driverStationKickerWheelControl() {
    kickerTargetSpeed = SmartDashboard.getNumber("Set Kicker Wheel Target Speed", 0);
    setKickerSpeed(kickerTargetSpeed);
  }

  public double getShooterRPM() {
    return ticksPer100msToRPM(shooterLeader.getSelectedSensorVelocity());
  }

  public double getFilteredShooterRPM() {
    return filteredShooterRPM;
  }

  public double getShooterTargetRPM() {
    return shooterTargetRPM;
  }

  public double getHoodMotorPosition() {
    return hoodEncoder.getPosition();
    //return 0;
  }

  public double getHoodMotorTargetPosition() {
    return targetAngle;
  }

  public boolean getIsReady() {
    return Math.abs(getShooterTargetRPM() - getShooterRPM()) <= ShooterConstants.RPMTolerance 
    && Math.abs(getHoodMotorPosition() - getHoodMotorTargetPosition()) <= ShooterConstants.angleTolerance
    && Math.abs(getShooterRPM()) >= 1000;
  }

  private double rpmToTicksPer100ms(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  private double ticksPer100msToRPM(double unitsPer100ms) {
    return unitsPer100ms * 600.0 / 2048.0;
  }

  private double rotationsToHoodAngleDegrees(double motorRotations) {
    return motorRotations * ShooterConstants.motorRotationsToHoodDegreesMoved + ShooterConstants.minAngle;
  }

  private double hoodAngleDegreesToRotations(double hoodAngleDegrees) {
    return (hoodAngleDegrees - ShooterConstants.minAngle) / ShooterConstants.motorRotationsToHoodDegreesMoved;
  }

  private double clamp(double value, double min, double max) {
    if (value < min) {
      return min;
    }
    return Math.min(value, max);
  }
}

/*
OLD STUFF:
Equation for shooter RPM given target area gotten from limelight:
2655 + -1379x + 866x^2
Where x is ta from limelight and output is shooter RPM

Equation for shooter servo position given area gotten from limelight:
0.418 + -1.77x + 0.85x^2

NEW STUFF:
hood motor position = 9
rpm = 2050
when camera tx = 19.5 degrees

hood motor position = 13
rpm = 3260
when camera tx = -8.2 degrees

tx dif = 27.7 degrees
hood pos dif = 4 degrees
hoodPos/tx = .144
m = -.144
y=mx+b
y=-.144x + b
13 = -.144(-8.2) + b
1.1808
b = 11.82
hoodPos = -.144(cameraTX) + 11.82

Equation for shooter RPM:
rpm = 2623 + -63.2x + 1.78x^2
*/
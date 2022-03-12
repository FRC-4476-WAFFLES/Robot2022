// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
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

  private double anglekP = ShooterConstants.anglekP;
  private double anglekI = ShooterConstants.anglekI;
  private double anglekD = ShooterConstants.anglekD;
  private double anglekF = ShooterConstants.anglekF;

  private double shooterTargetRPM = 0;
  private double kickerTargetSpeed = 0;
  private double targetAngle = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterLeader.configFactoryDefault();
    shooterFollower.configFactoryDefault();

    shooterFollower.follow(shooterLeader);
    shooterFollower.setInverted(InvertType.OpposeMaster);

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
    kickerWheel.setNeutralMode(NeutralMode.Brake);

    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setSmartCurrentLimit(20);
    hoodMotor.setIdleMode(IdleMode.kBrake);

    hoodEncoder.setPosition(0);

    hoodPIDController.setP(anglekP);
    hoodPIDController.setI(anglekI);
    hoodPIDController.setD(anglekD);
    hoodPIDController.setFF(anglekF);

    SmartDashboard.setDefaultNumber("Set Shooter Target Angle", -1.0);
    SmartDashboard.setDefaultNumber("Set Shooter Target RPM", 0);
    SmartDashboard.setDefaultNumber("Set Kicker Wheel Target Speed", 0);
    SmartDashboard.setDefaultNumber("Set Shooter Angle Offset", 0);

    SmartDashboard.setDefaultNumber("Shooter kP", shooterkP);
    SmartDashboard.setDefaultNumber("Shooter kI", shooterkI);
    SmartDashboard.setDefaultNumber("Shooter kD", shooterkD);
    SmartDashboard.setDefaultNumber("Shooter kF", shooterkF);

    SmartDashboard.setDefaultNumber("Shooter Angle kP", anglekP);
    SmartDashboard.setDefaultNumber("Shooter Angle kI", anglekI);
    SmartDashboard.setDefaultNumber("Shooter Angle kD", anglekD);
    SmartDashboard.setDefaultNumber("Shooter Angle kF", anglekF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Current RPM", ticksPer100msToRPM(shooterLeader.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter Raw Velocity", shooterLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Target RPM", shooterTargetRPM);
    SmartDashboard.putNumber("Shooter Target Angle", targetAngle);

    double shooterP = SmartDashboard.getNumber("Shooter kP", 0);
    double shooterI = SmartDashboard.getNumber("Shooter kI", 0);
    double shooterD = SmartDashboard.getNumber("Shooter kD", 0);
    double ShooterF = SmartDashboard.getNumber("Shooter kF", 0);

    double angleP = SmartDashboard.getNumber("Shooter Angle kP", 0);
    double angleI = SmartDashboard.getNumber("Shooter Angle kI", 0);
    double angleD = SmartDashboard.getNumber("Shooter Angle kD", 0);
    double angleF = SmartDashboard.getNumber("Shooter Angle kF", 0);

    updateShooterPID(shooterP, shooterI, shooterD, ShooterF);
    updateAnglePID(angleP, angleI, angleD, angleF);

    filteredShooterRPM = shooterRPMFilter.calculate(getShooterRPM());
  }

  public void updateShooterPID(double kP, double kI, double kD, double kF) {
    shooterLeader.config_kP(0, kP);
    shooterLeader.config_kI(0, kI);
    shooterLeader.config_kD(0, kD);
    shooterLeader.config_kF(0, kF);
  }

  public void updateAnglePID(double kP, double kI, double kD, double kF) {
    hoodPIDController.setP(kP);
    hoodPIDController.setI(kI);
    hoodPIDController.setD(kD);
    hoodPIDController.setFF(kF);
  }

  public void setKickerSpeed(double target) {
    target = clamp(target, -1, 1);
    kickerWheel.set(ControlMode.PercentOutput, target);
  }

  public void setShooterSpeed(double target) {
    target = clamp(target, -6000, 6000);
    shooterTargetRPM = target;
    shooterLeader.set(ControlMode.Velocity, rpmToTicksPer100ms(target));
  }

  public void setHoodAngle(double target) {
    targetAngle = target = clamp(target, ShooterConstants.minAngle, ShooterConstants.maxAngle);
    hoodPIDController.setReference(hoodAngleDegreesToRotations(target), ControlType.kPosition);
  }

  public void stop() {
    shooterLeader.set(ControlMode.PercentOutput, 0);
    kickerWheel.set(ControlMode.PercentOutput, 0);
    hoodMotor.stopMotor();
  }

  public void driverStationShooterRPM() {
    shooterTargetRPM = SmartDashboard.getNumber("Set Shooter Target RPM", 0);
    setShooterSpeed(shooterTargetRPM);
  }

  public void driverStationAngleControl() {
    targetAngle = SmartDashboard.getNumber("Set Shooter Target Angle", 0);
    setHoodAngle(targetAngle);
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

  public double getHoodAngleDegrees() {
    return rotationsToHoodAngleDegrees(hoodEncoder.getPosition());
  }

  public double getHoodTargetAngleDegrees() {
    return targetAngle;
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

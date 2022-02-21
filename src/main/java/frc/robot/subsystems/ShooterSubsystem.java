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

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.KickerWheelSpinup;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeader = new TalonFX(Constants.shooterSpinLeft);
  private final TalonFX shooterFollower = new TalonFX(Constants.shooterSpinRight);
  private final TalonSRX kickerWheel = new TalonSRX(Constants.kickerWheelSpin);
  private final Servo angleLeft = new Servo(1);
  private final Servo angleRight = new Servo(0);
  
  ShooterConstants shooterConstants = new ShooterConstants();

  private double kP = shooterConstants.kP;
  private double kI = shooterConstants.kI;
  private double kD = shooterConstants.kD;
  private double kF = shooterConstants.kF;
  private double kIzone = shooterConstants.kIzone;

  private double shooterTargetRPM = 0;
  private double shooterTargetAngle = 0;
  private double kickerTargetSpeed = 0;

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
    shooterLeader.configNominalOutputForward(0);
    shooterLeader.configNominalOutputReverse(0);
    shooterLeader.configPeakOutputForward(1);
    shooterLeader.configPeakOutputReverse(-1);
    shooterLeader.config_kP(0, kP);
    shooterLeader.config_kI(0, kI);
    shooterLeader.config_kD(0, kD);
    shooterLeader.config_kF(0, kF);
    shooterLeader.config_IntegralZone(0, kIzone);
    shooterLeader.configVoltageCompSaturation(12);
    shooterLeader.enableVoltageCompensation(true);
    shooterLeader.setNeutralMode(NeutralMode.Coast);
    shooterLeader.setInverted(true);

    kickerWheel.configFactoryDefault();
    kickerWheel.configContinuousCurrentLimit(30);
    kickerWheel.configPeakCurrentLimit(30);
    kickerWheel.enableCurrentLimit(true);
    kickerWheel.setNeutralMode(NeutralMode.Brake);

    angleLeft.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    angleRight.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    SmartDashboard.setDefaultNumber("Set Shooter Target Angle", -1.0);
    SmartDashboard.setDefaultNumber("Set Shooter Target RPM", 0);
    SmartDashboard.setDefaultNumber("Set Kicker Wheel Target Speed", 0);

    SmartDashboard.setDefaultNumber("Shooter kP", kP);
    SmartDashboard.setDefaultNumber("Shooter kI", kI);
    SmartDashboard.setDefaultNumber("Shooter kD", kD);
    SmartDashboard.setDefaultNumber("Shooter kF", kF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Current RPM", ticksPer100msToRPM(shooterLeader.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter Raw Velocity", shooterLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Target RPM", shooterTargetRPM);
    SmartDashboard.putNumber("Shooter Target Angle", shooterTargetAngle);

    double p = SmartDashboard.getNumber("Shooter kP", 0);
    double i =SmartDashboard.getNumber("Shooter kI", 0);
    double d = SmartDashboard.getNumber("Shooter kD", 0);
    double f = SmartDashboard.getNumber("Shooter kF", 0);

    updateShooterPID(p, i, d, f);
  }

  public void updateShooterPID(double kP, double kI, double kD, double kF) {
    shooterLeader.config_kP(0, kP);
    shooterLeader.config_kI(0, kI);
    shooterLeader.config_kD(0, kD);
    shooterLeader.config_kF(0, kF);
  }

  public void setKickerSpeed(double target) {
    target = clamp(target, -1, 1);
    kickerWheel.set(ControlMode.PercentOutput, target);
  }

  public void setShooterSpeed(double target) {
    target = clamp(target, -6000, 6000);
    shooterLeader.set(ControlMode.Velocity, rpmToTicksPer100ms(target));
  }

  public void setHoodAngle(double target) {
    target = clamp(target, -1.0, 1.0);
    angleLeft.setSpeed(target);
    angleRight.setSpeed(target);
  }

  public void stop() {
    shooterLeader.set(ControlMode.PercentOutput, 0);
    kickerWheel.set(ControlMode.PercentOutput, 0);
  }

  public void driverStationShooterRPM() {
    shooterTargetRPM = SmartDashboard.getNumber("Set Shooter Target RPM", 0);
    setShooterSpeed(shooterTargetRPM);
  }

  public void driverStationAngleControl() {
    shooterTargetAngle = SmartDashboard.getNumber("Set Shooter Target Angle", 0);
    setHoodAngle(shooterTargetAngle);
  }

  public void driverStationKickerWheelControl() {
    kickerTargetSpeed = SmartDashboard.getNumber("Set Kicker Wheel Target Speed", 0);
    setKickerSpeed(kickerTargetSpeed);
  }

  public double getShooterRPM() {
    return ticksPer100msToRPM(shooterLeader.getSelectedSensorVelocity());
  }

  private double rpmToTicksPer100ms(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  private double ticksPer100msToRPM(double unitsPer100ms) {
    return unitsPer100ms * 600.0 / 2048.0;
  }

  private double clamp(double value, double min, double max) {
    if (value < min) {
      return min;
    }
    return Math.min(value, max);
  }
}

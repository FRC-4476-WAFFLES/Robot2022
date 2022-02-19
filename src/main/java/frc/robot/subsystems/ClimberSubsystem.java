// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climbLeft = new TalonFX(Constants.climbLeft);
  private final TalonFX climbRight = new TalonFX(Constants.climbRight);

  private final Timer timer = new Timer();

  private int targetSetpoint = 0;
  private double previousTime = 0;
  private double previousLoopTime = 0;

  
  public ClimberSubsystem() {
    climbLeft.configFactoryDefault();
    climbLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbLeft.setInverted(TalonFXInvertType.Clockwise);
    climbLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbLeft.config_kP(0, 0.1);
    climbLeft.config_kI(0, 0);
    climbLeft.config_kD(0, 0.1);
    climbLeft.setNeutralMode(NeutralMode.Brake);
    climbLeft.setSelectedSensorPosition(0);

    climbRight.configFactoryDefault();
    climbRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbRight.setInverted(TalonFXInvertType.CounterClockwise);
    climbRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbRight.config_kP(0, 0.1);
    climbRight.config_kI(0, 0);
    climbRight.config_kD(0, 0.1);
    climbRight.setNeutralMode(NeutralMode.Brake);
    climbRight.setSelectedSensorPosition(0);

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //climbLeft.set(ControlMode.Position, targetSetpoint);
    //climbRight.set(ControlMode.Position, targetSetpoint);

    previousLoopTime = timer.get() - previousTime;
    previousTime = timer.get();

    SmartDashboard.putNumber("Climber Target", targetSetpoint);
    SmartDashboard.putNumber("Climber Left Position", climbLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Climber Right Position", climbRight.getSelectedSensorPosition());
  }

  public void moveClimberSetpoint(double amountToMove) {
    targetSetpoint += amountToMove;
    /*
    if (targetSetpoint < 0) {
      targetSetpoint = 0;
    }*/
  }

  public void moveClimberWithAnalogStick(double analogStickValue) {
    moveClimberSetpoint(analogStickValue * previousLoopTime * 80000);
  }

  public void stopClimber() {
    climbLeft.set(ControlMode.PercentOutput, 0);
    climbRight.set(ControlMode.PercentOutput, 0);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climbLeft = new TalonFX(Constants.climbLeft);
  private final TalonFX climbRight = new TalonFX(Constants.climbRight);
  private final TalonFX climbPivotLeft = new TalonFX(Constants.climbPivotLeft);
  private final TalonFX climbPivotRight = new TalonFX(Constants.climbPivotRight);

  private final int telHookMaxExt = 260000;
  private final int telHookMinExt = -6000;
  private final int telHookAboveBar = 60000;
  private final int telHookPulledOffBar = 160000;

  private final int rotHookOutOfWay = -29000;
  private final int rotHookOnBar = 0;
  private final int rotHookRobotToNextBar = 57000;
  private final int rotHookRobotOnNextBar = 19000;

  private final ClimberState climbStates[] = new ClimberState[] {
    // Start
    new ClimberState(0, 0),

    // Go to mid bar
    new ClimberState(telHookMaxExt, rotHookOutOfWay),
    new ClimberState(telHookMinExt, rotHookOutOfWay),
    new ClimberState(telHookMinExt, rotHookOnBar),

    // Go to high bar
    new ClimberState(telHookAboveBar, rotHookOnBar),
    new ClimberState(telHookAboveBar, rotHookRobotToNextBar),
    new ClimberState(telHookMaxExt, rotHookRobotToNextBar),
    new ClimberState(telHookMaxExt, rotHookRobotOnNextBar),
    new ClimberState(telHookPulledOffBar, rotHookRobotOnNextBar),
    new ClimberState(telHookMinExt, rotHookOutOfWay),
    new ClimberState(telHookMinExt, rotHookOnBar),

    // Go to traverse bar
    new ClimberState(telHookAboveBar, rotHookOnBar),
    new ClimberState(telHookAboveBar, rotHookRobotToNextBar),
    new ClimberState(telHookMaxExt, rotHookRobotToNextBar),
    new ClimberState(telHookMaxExt, rotHookRobotOnNextBar),
    new ClimberState(telHookPulledOffBar, rotHookRobotOnNextBar),
    new ClimberState(telHookMinExt, rotHookOutOfWay),
    new ClimberState(telHookMinExt, rotHookOnBar)

    // Max telescoping tube extention = 260 000 ticks with 20:1 gearbox
    // Rotating hooks out of way = -29 000 ticks
    // Rotating hooks moving to next bar = 57 000 ticks
    // Rotating hooks with telescoping hooks on next bar = 19 000 ticks
  };

  private int currentSetpoint = 0;

  private final Timer timer = new Timer();

  private int targetSetpoint = 0;
  private double previousTime = 0;
  private double previousLoopTime = 0;

  private int targetPivotSetpoint = 0;

  public ClimberSubsystem() {
    climbLeft.configFactoryDefault();
    climbLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbLeft.setInverted(TalonFXInvertType.Clockwise);
    climbLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbLeft.config_kP(0, 0.1);
    climbLeft.config_kI(0, 0);
    climbLeft.config_kD(0, 0.0);
    climbLeft.setNeutralMode(NeutralMode.Brake);
    climbLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    climbLeft.setSelectedSensorPosition(0);

    climbRight.configFactoryDefault();
    climbRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbRight.setInverted(TalonFXInvertType.CounterClockwise);
    climbRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbRight.config_kP(0, 0.1);
    climbRight.config_kI(0, 0);
    climbRight.config_kD(0, 0.0);
    climbRight.setNeutralMode(NeutralMode.Brake);
    climbRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    climbRight.setSelectedSensorPosition(0);

    climbPivotLeft.configFactoryDefault();
    climbPivotLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbPivotLeft.setInverted(TalonFXInvertType.Clockwise);
    climbPivotLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbPivotLeft.config_kP(0, 0.1);
    climbPivotLeft.config_kI(0, 0);
    climbPivotLeft.config_kD(0, 0);
    climbPivotLeft.setNeutralMode(NeutralMode.Brake);
    climbPivotLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    climbPivotLeft.setSelectedSensorPosition(0);

    climbPivotRight.configFactoryDefault();
    climbPivotRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbPivotRight.setInverted(TalonFXInvertType.CounterClockwise);
    climbPivotRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbPivotRight.config_kP(0, 0.1);
    climbPivotRight.config_kI(0, 0);
    climbPivotRight.config_kD(0, 0);
    climbPivotRight.setNeutralMode(NeutralMode.Brake);
    climbPivotRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
    climbPivotRight.setSelectedSensorPosition(0);

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbLeft.set(ControlMode.Position, targetSetpoint);
    climbRight.set(ControlMode.Position, targetSetpoint);
    climbPivotLeft.set(ControlMode.Position, targetPivotSetpoint);
    climbPivotRight.set(ControlMode.Position, targetPivotSetpoint);

    previousLoopTime = timer.get() - previousTime;
    previousTime = timer.get();

    SmartDashboard.putNumber("Current Setpoint Number", currentSetpoint);

    SmartDashboard.putNumber("Climber Target", targetSetpoint);
    SmartDashboard.putNumber("Climber Left Position", climbLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Climber Right Position", climbRight.getSelectedSensorPosition());

    SmartDashboard.putNumber("Climber Pivot Target", targetPivotSetpoint);
    SmartDashboard.putNumber("Climber Pivot Left Position", climbPivotLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Climber Pivot Right Position", climbPivotRight.getSelectedSensorPosition());
  }

  public void moveClimberSetpoint(double amountToMove) {
    targetSetpoint += amountToMove;
  }

  public void setClimberSetpoint(int setpoint) {
    targetSetpoint = setpoint;
  }

  public void moveClimberPivotSetpoint(double amountToMove) {
    targetPivotSetpoint += amountToMove;
  }

  public void setClimberPivotSetpoint(int setpoint) {
    targetSetpoint = setpoint;
  }

  public void moveClimberWithAnalogStick(double analogStickValue) {
    moveClimberSetpoint(analogStickValue * previousLoopTime * 160000);
  }

  public void moveCLimberPivotWithAnalogStick(double analogStickValue) {
    moveClimberPivotSetpoint(analogStickValue * previousLoopTime * 80000);
  }
  
  public void nextSetpoint() {
    currentSetpoint++;
    currentSetpoint = (int) (clamp(currentSetpoint, 0, climbStates.length));
    targetSetpoint = climbStates[currentSetpoint].climbTargetSetpoint;
    targetPivotSetpoint = climbStates[currentSetpoint].climbPivotTargetSetpoint;
  }

  public void previousSetpoint() {
    currentSetpoint--;
    currentSetpoint = (int) (clamp(currentSetpoint, 0, climbStates.length));
    targetSetpoint = climbStates[currentSetpoint].climbTargetSetpoint;
    targetPivotSetpoint = climbStates[currentSetpoint].climbPivotTargetSetpoint;
  }

  public void stopClimber() {
    climbLeft.set(ControlMode.PercentOutput, 0);
    climbRight.set(ControlMode.PercentOutput, 0);
    climbPivotLeft.set(ControlMode.PercentOutput, 0);
    climbPivotRight.set(ControlMode.PercentOutput, 0);
  }

  private double clamp(double value, double min, double max) {
    if (value < min) {
      return min;
    }
    return Math.min(value, max);
  }

  private static final class ClimberState {
    final int climbTargetSetpoint;
    final int climbPivotTargetSetpoint;

    private ClimberState(int climbTargetSetpoint, int climbPivotTargetSetpoint) {
      this.climbTargetSetpoint = climbTargetSetpoint;
      this.climbPivotTargetSetpoint = climbPivotTargetSetpoint;
    }
  }
}

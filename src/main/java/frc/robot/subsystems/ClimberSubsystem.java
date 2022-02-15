// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberPositions;

import java.util.Map;

public class ClimberSubsystem extends SubsystemBase {
  /*
  public final double tolerance = 5;

  public static Map<Integer, ClimberConstants> climberSetpoints = Map.ofEntries(
    Map.entry(0, new ClimberConstants(0, 0, 0, 0)),
    Map.entry(1, new ClimberConstants(1, 1, 1, 1))
);*/

  private final TalonFX climbLeft = new TalonFX(Constants.climbLeft);
  private final TalonFX climbRight = new TalonFX(Constants.climbRight);
  //private final TalonFX climbPivotLeft = new TalonFX(Constants.climbPivotLeft);
  //private final TalonFX climbPivotRight = new TalonFX(Constants.climbPivotRight);

  // Keep track of which climbing state
  //private int state = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbLeft.configFactoryDefault();
    climbLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    climbRight.configFactoryDefault();
    climbRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbRight.follow(climbLeft);
    climbRight.setInverted(InvertType.OpposeMaster);

    climbLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
    climbLeft.config_kP(0, 0.5);
    climbLeft.config_kI(0, 0);
    climbLeft.config_kD(0, 0.1);
    climbLeft.setNeutralMode(NeutralMode.Brake);

    /*
    climbPivotLeft.configFactoryDefault();
    climbPivotLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));

    climbPivotRight.configFactoryDefault();
    climbPivotRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03)); */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: this is not done
    /*
    switch (state) {
      case 0:
        climbLeft.set(TalonFXControlMode.Disabled, 0);
        climbRight.set(TalonFXControlMode.Disabled, 0);

        // TODO: Figure out where pivots should be positioned when not being used
        //climbPivotLeft.set(TalonFXControlMode.Disabled, 0);
        //climbPivotRight.set(TalonFXControlMode.Disabled, 0);

        break;
      case 1:
        // wait for input
        break;
      case 2:
        break;
    }*/
  }

  public void setTarget(ClimberPositions targetPosition) {
    // Set the target position of the climber motors to the desired climber position
    climbLeft.set(ControlMode.Position, ClimberPositions.getEncoderValue(targetPosition));
  }

  /*
  public boolean atSetpoints() {
    return climbLeft.getClosedLoopError() < tolerance
    && climbRight.getClosedLoopError() < tolerance
    && climbPivotLeft.getClosedLoopError() < tolerance
    && climbPivotRight.getClosedLoopError() < tolerance;
  } */
}

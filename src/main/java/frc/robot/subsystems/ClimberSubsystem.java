// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climbLeft = new TalonFX(Constants.climbLeft);
  private final TalonFX climbRight = new TalonFX(Constants.climbRight);
  private final TalonFX climbPivotLeft = new TalonFX(Constants.climbPivotLeft);
  private final TalonFX climbPivotRight = new TalonFX(Constants.climbPivotRight);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbLeft.configFactoryDefault();
    climbLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));

    climbRight.configFactoryDefault();
    climbRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));

    climbPivotLeft.configFactoryDefault();
    climbPivotLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));

    climbPivotRight.configFactoryDefault();
    climbPivotRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeader = new TalonFX(Constants.shooterSpin1);
  private final TalonFX shooterFollower = new TalonFX(Constants.shooterSpin2);
  private final TalonSRX kickerWheel = new TalonSRX(Constants.kickerWheelSpin);
  private final Servo angle1 = new Servo(Constants.shooterHoodAngle1);
  private final Servo angle2 = new Servo(Constants.shooterHoodAngle2);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterLeader.configFactoryDefault();
    shooterFollower.configFactoryDefault();

    shooterFollower.follow(shooterLeader);
    shooterFollower.setInverted(InvertType.OpposeMaster);

    shooterLeader.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));

    kickerWheel.configFactoryDefault();
    kickerWheel.configContinuousCurrentLimit(30);
    kickerWheel.configPeakCurrentLimit(30);
    kickerWheel.enableCurrentLimit(true);

    angle1.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    angle2.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

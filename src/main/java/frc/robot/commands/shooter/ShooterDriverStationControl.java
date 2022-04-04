// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

public class ShooterDriverStationControl extends CommandBase {
  /** Creates a new DriverStationShooterControl. */
  public ShooterDriverStationControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setLEDMode(CameraLEDMode.On);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooterSubsystem.driverStationAngleControl();
    vision.setLEDMode(CameraLEDMode.On);
    shooterSubsystem.setHoodAngle(3.78 * vision.getHorisontalFieldDistanceToGoal() - 0.45);
    shooterSubsystem.driverStationShooterRPM();
    shooterSubsystem.driverStationKickerWheelControl();
    //shooterSubsystem.driverStationAngleOffsetControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

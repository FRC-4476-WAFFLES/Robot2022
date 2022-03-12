// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

public class ShooterVisionSetup extends CommandBase {
  /** Creates a new ShooterSetup. */
  public ShooterVisionSetup() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterTargetRPM;
    double shooterTargetAngle;
    vision.setLEDMode(CameraLEDMode.On);
    if (vision.getHasTarget()) {
      double limelightTA = vision.getFilteredArea();
      shooterTargetRPM = 2655 - 1379 * limelightTA + 866 * Math.pow(limelightTA, 2);
      shooterTargetAngle = 0.418 - 1.77 * limelightTA + 0.85 * Math.pow(limelightTA, 2);
    } else {
      shooterTargetRPM = 1800;
      shooterTargetAngle = -0.7;
    }

    shooterSubsystem.setShooterSpeed(shooterTargetRPM);
    shooterSubsystem.setHoodAngle(shooterTargetAngle);
    shooterSubsystem.setKickerSpeed(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(shooterSubsystem.getShooterTargetRPM() - shooterSubsystem.getFilteredShooterRPM()) <= ShooterConstants.RPMTolerance) 
    && (Math.abs(shooterSubsystem.getHoodAngleDegrees() - shooterSubsystem.getHoodTargetAngleDegrees()) <= ShooterConstants.angleTolerance);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

public class ShooterVisionSetup extends CommandBase {
  private Timer timeSinceLastHasTarget = new Timer();
  private Timer startTimer = new Timer();
  private double savedLimelightTY = 0; // Set the savedLimelightTY to be a value to get the hood to go to fender shot angle

  /** Creates a new ShooterSetup. */
  public ShooterVisionSetup() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.err.println("Intializing shooter ready");
    timeSinceLastHasTarget.reset();
    timeSinceLastHasTarget.start();
    startTimer.reset();
    startTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.err.println("Readying Shooter");
    double shooterTargetRPM;
    double shooterTargetAngle;
    vision.setLEDMode(CameraLEDMode.On);
    
    if (vision.getHasTarget()) {
      //System.err.println("Shooter has target");
      timeSinceLastHasTarget.reset();
      double limelightTY = vision.getFilteredVertical();
      savedLimelightTY = limelightTY;
      shooterTargetRPM = 2702 - 56.7 * limelightTY + 1.26 * Math.pow(limelightTY, 2);
      shooterTargetAngle = -0.144 * limelightTY + 11.82;

    } else if (timeSinceLastHasTarget.get() < 0.5) {
      if (startTimer.get() < 0.5) {
        //System.err.println("1");
        shooterTargetRPM = 2050.0;
        shooterTargetAngle = 0.0;
      } else {
        //System.err.println("2");
        shooterTargetRPM = 2702 - 56.7 * savedLimelightTY + 1.26 * Math.pow(savedLimelightTY, 2); // was 2702.0 for c value, a = 1.26, b = 56.7
        shooterTargetAngle = -0.144 * savedLimelightTY + 11.82;
      }

    } else {
      //System.err.println("3");
      shooterTargetRPM = 2050.0;
      shooterTargetAngle = 0.0;
    }

    shooterSubsystem.setShooterSpeed(shooterTargetRPM);
    shooterSubsystem.setHoodAngle(shooterTargetAngle);
    shooterSubsystem.setKickerSpeed(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.err.println("Shooter Ready");
    vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooterSubsystem.getShooterRPM() - shooterSubsystem.getShooterTargetRPM()) <= ShooterConstants.RPMTolerance
    && Math.abs(shooterSubsystem.getHoodMotorPosition() - shooterSubsystem.getHoodMotorTargetPosition()) <= ShooterConstants.angleTolerance;
  }
}

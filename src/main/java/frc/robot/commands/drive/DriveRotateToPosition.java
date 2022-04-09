// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class DriveRotateToPosition extends CommandBase {
  private final PIDController turnController = new PIDController(-4.0, 0, -0.2);
  private final double targetHeading;
  
  /** Creates a new DriveRotateToPosition. */
  public DriveRotateToPosition(double targetHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    this.targetHeading = targetHeading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = driveSubsystem.getOdometryLocation().getRotation().getRadians();
    double optimizedTargetAngle = currentHeading + Rotation2d.fromDegrees(targetHeading).minus(new Rotation2d(currentHeading)).getRadians();
    turnController.setSetpoint(optimizedTargetAngle);
    double turnSpeed = turnController.calculate(currentHeading);
    driveSubsystem.robotDrive(0.0, 0.0, turnSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetHeading - driveSubsystem.getOdometryLocation().getRotation().getDegrees()) < 5;
  }
}

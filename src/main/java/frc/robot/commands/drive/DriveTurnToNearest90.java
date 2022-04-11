// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;

import static frc.robot.RobotContainer.*;

public class DriveTurnToNearest90 extends CommandBase {
  private final PIDController turnController = new PIDController(-4.0, 0, -0.2);
  private double targetHeading = 0;

  /** Creates a new TurnToNearest90. */
  public DriveTurnToNearest90() {
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = driveSubsystem.getOdometryLocation().getRotation().getDegrees();
    currentHeading = Math.round(currentHeading * 5) / 5; // Ryan told me to do this to round a number to half a decimal (his words)

    if (currentHeading < -135) {
      targetHeading = -180;
    } else if (currentHeading < -45) {
      targetHeading = -90;
    } else if (currentHeading < 45) {
      targetHeading = 0;
    } else if (currentHeading < 135) {
      targetHeading = 90;
    } else {
      targetHeading = 180;
    }

    //targetHeading += 45; // IDK why this is needed, java bad

    double optimizedTargetAngle = Math.toRadians(currentHeading) + Rotation2d.fromDegrees(targetHeading).minus(Rotation2d.fromDegrees(currentHeading)).getRadians();
    turnController.setSetpoint(optimizedTargetAngle);
    double turnSpeed = turnController.calculate(Math.toRadians(currentHeading));

    double forward = leftJoystick.getY();
    double right = leftJoystick.getX();

    forward = MathUtil.applyDeadband(forward, 0.1);
    right = MathUtil.applyDeadband(right, 0.1);

    forward *= SwerveConstants.maxAttainableSpeedMetersPerSecond;
    right *= SwerveConstants.maxAttainableSpeedMetersPerSecond;

    driveSubsystem.robotDrive(forward, right, turnSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getOdometryLocation().getRotation().getDegrees() - targetHeading) < 5;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;

import static frc.robot.RobotContainer.*;

import java.util.function.Supplier;

public class DriveAndPIDTurn extends CommandBase {
  private final Supplier<Rotation2d> targetHeading;
  private final PIDController turnController;
  private final double headingTolerance;
  
  /** Creates a new DriveAndPIDTurn. 
   * @param targetHeading Rotation2d supplier for the heading to go to
   * @param turnController PIDController object for regulating turning
   * @param headingTolerance Tolerance, in degrees, that the command will allow
   * @param requirements The requirements to add. Automatically adds <code>driveSubsystem</code>
  */
  public DriveAndPIDTurn(Supplier<Rotation2d> targetHeading, PIDController turnController, double headingTolerance, Subsystem... requirements) {
    // Use addRequirements() here to declare subsystem dependencies.
    Subsystem[] fullRequirements = new Subsystem[requirements.length + 1];
    for (int x = 0; x <= requirements.length; x++) {
      fullRequirements[x] = x < requirements.length ? requirements[x] : driveSubsystem;
    }

    addRequirements(fullRequirements);

    this.targetHeading = targetHeading;
    this.turnController = turnController;
    this.headingTolerance = headingTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = driveSubsystem.getOdometryLocation().getRotation().getRadians();
    double optimizedTargetHeading = currentHeading + targetHeading.get().minus(new Rotation2d(currentHeading)).getRadians();
    turnController.setSetpoint(optimizedTargetHeading);
    double turnSpeed = turnController.calculate(currentHeading);

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
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetHeading.get().getDegrees() - driveSubsystem.getOdometryLocation().getRotation().getDegrees()) < headingTolerance;
  }
}

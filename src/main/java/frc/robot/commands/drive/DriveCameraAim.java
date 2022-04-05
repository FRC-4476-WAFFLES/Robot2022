// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

public class DriveCameraAim extends CommandBase {
  PIDController turnController = new PIDController(-4.0, 0, -0.2);
  Trigger[] overrideOdometryTriggers;

  /** Creates a new DriveCameraAim. */
  public DriveCameraAim(Trigger[] overrideOdometryTriggers) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    this.overrideOdometryTriggers = overrideOdometryTriggers;
  }

  public DriveCameraAim() {
    this(new Trigger[] {
      new Trigger()
    });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.err.println("Drive aim supposedly commencing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = driveSubsystem.getOdometryLocation().getRotation().getRadians();
    boolean overrideOdometry = false;
    //System.err.println("Drive aim supposedly executing");
    vision.setLEDMode(CameraLEDMode.On);

    for (Trigger trigger : overrideOdometryTriggers) {
      if (trigger.get()) {
        overrideOdometry = true;
      }
    }

    double targetHeading = Math.atan2(-driveSubsystem.getGoalOdometryLocation().getY(), driveSubsystem.getGoalOdometryLocation().getX());

    double optimizedTargetAngle = currentHeading + new Rotation2d(targetHeading).minus(new Rotation2d(currentHeading)).getRadians();

    if (Math.abs(optimizedTargetAngle - currentHeading) <= Math.toRadians(20) || vision.getHasTarget() || overrideOdometry) {
      targetHeading = currentHeading - Math.toRadians(vision.getFilteredHorizontal());
      optimizedTargetAngle = currentHeading + new Rotation2d(targetHeading).minus(new Rotation2d(currentHeading)).getRadians();
    }

    turnController.setSetpoint(optimizedTargetAngle);

    double turnSpeed = turnController.calculate(currentHeading);

    double forward = leftJoystick.getY();
    double right = leftJoystick.getX();

    forward = MathUtil.applyDeadband(forward, 0.05);
    right = MathUtil.applyDeadband(right, 0.05);

    forward *= Constants.SwerveConstants.maxAttainableSpeedMetersPerSecond;
    right *= Constants.SwerveConstants.maxAttainableSpeedMetersPerSecond;

    SmartDashboard.putNumber("Left Joystick Y", forward);
    SmartDashboard.putNumber("Left Joystick X", right);

    driveSubsystem.robotDrive(forward, right, turnSpeed, true);

    SmartDashboard.putNumber("Drive Current Rotation Error (Radians)", Math.toRadians(vision.getFilteredHorizontal()));
    SmartDashboard.putNumber("Drive Target Heading", targetHeading);
    SmartDashboard.putNumber("Drive Current Heading", driveSubsystem.getOdometryLocation().getRotation().getRadians());
    SmartDashboard.putNumber("Drive PID Turn Speed", turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.err.println("Drive aim supposedly finished");
    driveSubsystem.stop();
    vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(vision.getHorizontal()) < 1.0 && vision.getHasTarget();
  }
}

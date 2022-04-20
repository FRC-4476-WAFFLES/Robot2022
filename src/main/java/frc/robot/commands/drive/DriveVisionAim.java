// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

import java.util.function.Supplier;

public class DriveVisionAim extends DriveAndPIDTurn {
  static Supplier<Rotation2d> targetHeading = driveSubsystem.getOdometryLocation()::getRotation;
  static final PIDController turnController = new PIDController(-5.5, 0, -0.4); // -5.0, 0, -0.2
  Trigger[] overrideOdometryTriggers;

  /** Creates a new DriveVisionAim. */
  public DriveVisionAim(Trigger[] overrideOdometryTriggers) {
    super(targetHeading, turnController, SwerveConstants.aimToleranceDegrees);
    this.overrideOdometryTriggers = overrideOdometryTriggers;
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

    if (vision.getHasTarget() || overrideOdometry) {
      targetHeading = () -> new Rotation2d(currentHeading - Math.toRadians(vision.getFilteredHorizontal()));
    } else {
      targetHeading = () -> new Rotation2d(Math.atan2(-driveSubsystem.getGoalOdometryLocation().getY(), driveSubsystem.getGoalOdometryLocation().getX()));
    }

    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished() && vision.getHasTarget();
  }
}

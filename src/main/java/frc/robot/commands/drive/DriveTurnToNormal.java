// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.RobotContainer.*;

public class DriveTurnToNormal extends DriveAndPIDTurn {
  static Supplier<Rotation2d> targetHeading = () -> new Rotation2d();
  static final PIDController turnController = new PIDController(-4.0, 0, -0.2); // -5.0, 0, -0.2

  /** Creates a new DriveTurnToNormal. */
  public DriveTurnToNormal() {
    super(targetHeading, turnController, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = driveSubsystem.getOdometryLocation().getRotation().getDegrees();
    double targetHeadingDegrees;
    currentHeading = Math.round(currentHeading * 5) / 5; // Ryan told me to do this to round a number to half a decimal (his words)

    if (currentHeading < -135) {
      targetHeadingDegrees = -180;
    } else if (currentHeading < -45) {
      targetHeadingDegrees = -90;
    } else if (currentHeading < 45) {
      targetHeadingDegrees = 0;
    } else if (currentHeading < 135) {
      targetHeadingDegrees = 90;
    } else {
      targetHeadingDegrees = 180;
    }

    targetHeading = () -> Rotation2d.fromDegrees(targetHeadingDegrees);

    super.execute();
  }
}

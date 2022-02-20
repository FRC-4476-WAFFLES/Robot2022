// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRouteAlpha extends SequentialCommandGroup {
  /** Creates a new AutonomousAlpha. */
  public AutonomousRouteAlpha() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetToAutoStartingPosition(),
      // new FenderHighShotComplete()
      new DriveAuto(
        1.5,
        Rotation2d.fromDegrees(-110), 
        new Pose2d(0, 0, Rotation2d.fromDegrees(-110 + 180)),
        new Pose2d(-0.3, 0.3, Rotation2d.fromDegrees(-110 + 180)),
        new Pose2d(-0.6, 2.05, Rotation2d.fromDegrees(-110 + 180))));
  }
}
//Start at 111 degrees
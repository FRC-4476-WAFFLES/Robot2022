// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveAuto.SwervePath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAutoPath extends SequentialCommandGroup {
  /** Creates a new AutonomousAlpha. */
  public ThreeBallAutoPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetToRightAutoStartingPosition(),
      // new FenderHighShotComplete()
      // Get to first ball
      new DriveAuto(
        new SwervePath(0, 0, -110, -110 + 180)
        .waypoint(-0.15, 1.025, -110)
        .finish(-0.3, 2.05, -88, -88 + 180, 3.0)),

      // Get to second ball
      new DriveAuto(
        new SwervePath(-0.3, 2.05, -88, -88)
        .waypoint(-1.0, 1.3, -160)
        .finish(-2.27, 0.7, -160, -160, 3.0)
      ),

      // Get to shooting location
      new DriveAuto(
        new SwervePath(-2.27, 0.7, -160, -160 + 180)
        .waypoint(-1.0, 1.0, -110)
        .finish(0, 0, -110, -110, 3.0)));
  }
}
//Start at 111 degrees
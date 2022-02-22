// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveAuto.SwervePath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAutoPath extends SequentialCommandGroup {
  /** Creates a new TheAutoToRuleAllAutos. */
  public FiveBallAutoPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetToRightAutoStartingPosition(),
      // new FenderHighShotComplete()
      // Get to first ball
      new DriveAuto(
        new SwervePath(0, 0, -110, -110 + 180)
        .waypoint(-0.3, 0.3, -110)
        .finish(-0.3, 2.05, -88, -88 + 180, 3)),

      // Get to second ball
      new DriveAuto(
        new SwervePath(-0.3, 2.05, -88, -88 + 180)
        .finish(-2.2, 1.00, -169, -169 + 180, 3)),
      
      /*
      new DriveAuto(
        3.0, 
        Rotation2d.fromDegrees(-120.0), 
        new Pose2d(-2.2, 1.00, Rotation2d.fromDegrees(-169.0 + 180)),
        new Pose2d(-2.2, 1.00, Rotation2d.fromDegrees(-120.0 + 180))),
      */

      new WaitCommand(2),

      new DriveAuto(
        new SwervePath(-2.2, 1.0, -169, -169 + 180)
        .finish(-6.67, 0.90, -176, -176 + 180, 3))
    ); // -6.67, 0.90, -176.5
  }
}

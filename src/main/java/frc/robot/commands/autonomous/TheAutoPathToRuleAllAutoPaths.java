// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TheAutoPathToRuleAllAutoPaths extends SequentialCommandGroup {
  /** Creates a new TheAutoToRuleAllAutos. */
  public TheAutoPathToRuleAllAutoPaths() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetToAutoStartingPosition(),
      // new FenderHighShotComplete()
      // Get away from fender
      new DriveAuto(
        3.0, 
        Rotation2d.fromDegrees(-110),
        new Pose2d(0, 0, Rotation2d.fromDegrees(-110 + 180)),
        new Pose2d(-0.3, 0.3, Rotation2d.fromDegrees(-110 + 180))),

      // Get to first ball
      new DriveAuto(
        3.0,
        Rotation2d.fromDegrees(-88.0),
        new Pose2d(-0.3, 0.3, Rotation2d.fromDegrees(-110 + 180)),
        new Pose2d(-0.3, 2.05, Rotation2d.fromDegrees(-88.0 + 180))),
      
      // Get to second ball
      new DriveAuto(
        3.0,
        Rotation2d.fromDegrees(-169.0), 
        new Pose2d(-0.3, 2.05, Rotation2d.fromDegrees(-86.0)),
        new Pose2d(-2.2, 1.00, Rotation2d.fromDegrees(-169.0 + 180)))/*,

      new DriveAuto(
        3.0, 
        Rotation2d.fromDegrees(-120.0), 
        new Pose2d(-2.2, 1.00, Rotation2d.fromDegrees(-169.0 + 180)),
        new Pose2d(-2.2, 1.00, Rotation2d.fromDegrees(-120.0 + 180)))*/,

      new WaitCommand(2),

      new DriveAuto(
        3.0,
        Rotation2d.fromDegrees(-176.5), 
        new Pose2d(-2.2, 1.00, Rotation2d.fromDegrees(-120.0 + 180)),
        new Pose2d(-6.67, 0.90, Rotation2d.fromDegrees(-176.5 + 180)))
    ); // -6.67, 0.90, -176.5
  }
}

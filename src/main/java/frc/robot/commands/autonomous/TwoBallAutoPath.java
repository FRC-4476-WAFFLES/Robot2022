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
public class TwoBallAutoPath extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoPath. */
  public TwoBallAutoPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetToLeftAutoStartingPosition(),

      new DriveAuto(new SwervePath(0, 0, 160, 160)
          .waypoint(-0.3, -0.3, 160)
          .finish(-1.7, -1.6, 160, 160, 3.0)),

      new DriveAuto(new SwervePath(-1.7, -1.6, 160, 160)
          .finish(0, 0, 160, 160, 3.0))
    );
  }
}

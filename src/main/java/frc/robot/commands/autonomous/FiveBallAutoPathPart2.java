// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveAuto.SwervePath;
import frc.robot.commands.utility.ProxyVariableCommand;
import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAutoPathPart2 extends SequentialCommandGroup {
  /** Creates a new FiveBallAutoPathPart2. */
  public FiveBallAutoPathPart2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new InstantCommand(() -> new DriveAuto(
      //   new SwervePath(-0.3, 2.05, -88, RobotContainer.driveSubsystem.getOdometryLocation().getRotation().getDegrees())
      //   .waypoint(-1.0, 1.3, -160)
      //   .finish(-2.27, 0.7, -160, -160, 3.0)
      // ).schedule());

      /*
      new DriveAuto(
        new SwervePath(-2.27, 0.7, -160, -160)
        .finish(-6.5, 1.41, -139, -139, 4.0)),*/
      new ProxyVariableCommand(() -> new DriveAuto(
        new SwervePath(-2.27, 0.7, driveSubsystem.getOdometryLocation().getRotation().getDegrees(), -160)
        .finish(-6.5, 1.41, -139, -139, 4.0)
      )),

      new WaitCommand(1.0),

      new DriveAuto(
        new SwervePath(-6.5, 1.41, -139, -139 + 180)
        .finish(-2.27, 0.7, -160, -160 + 180, 4.0)
      )
    );
  }
}

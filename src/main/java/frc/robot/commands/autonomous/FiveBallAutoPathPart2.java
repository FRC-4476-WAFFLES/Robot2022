// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveAuto.SwervePath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAutoPathPart2 extends SequentialCommandGroup {
  /** Creates a new FiveBallAutoPathPart2. */
  public FiveBallAutoPathPart2(Supplier<Rotation2d> startHeading) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveAuto(
        new SwervePath(-2.2, 1.0, startHeading.get().getDegrees(), startHeading.get().getDegrees() + 180)
        .finish(-6.67, 0.90, -176, -176 + 180, 3)),
      
      // Wait for intaking to finish
      new WaitCommand(2.0),
      
      // Theoretically this can be removed, but then we would be doing a half-court shot
      new DriveAuto(
        new SwervePath(-6.67, 0.90, -176, -176)
        .finish(-2.2, 1.0, -169, -169, 3))
    );
  }
}

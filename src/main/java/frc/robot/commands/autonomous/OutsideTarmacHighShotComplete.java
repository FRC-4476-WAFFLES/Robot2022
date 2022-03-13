// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.Shoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OutsideTarmacHighShotComplete extends SequentialCommandGroup {
  /** Creates a new OutsideTarmacHighShotComplete. */
  public OutsideTarmacHighShotComplete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OutsideTarmacHighShotSetup(), new Shoot().withTimeout(3.0)
    );
  }
}
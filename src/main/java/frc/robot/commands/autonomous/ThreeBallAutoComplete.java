// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeAuto;
import frc.robot.commands.intake.IntakeDeploy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAutoComplete extends SequentialCommandGroup {
  /** Creates a new ThreeBallAutoComplete. */
  public ThreeBallAutoComplete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeDeploy().withTimeout(3.0), 
      new FenderHighShotComplete(), 
      new ThreeBallAutoPath().deadlineWith(new IntakeAuto(1.0)), 
      new WaitCommand(0.5), 
      new FenderHighShotComplete());
  }
}

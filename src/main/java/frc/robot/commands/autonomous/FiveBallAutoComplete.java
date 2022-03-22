// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveCameraAim;
import frc.robot.commands.intake.IntakeAuto;
import frc.robot.commands.intake.IntakeDeploy;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterVisionSetup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAutoComplete extends SequentialCommandGroup {
  /** Creates a new FiveBallAutoComplete. */
  public FiveBallAutoComplete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeDeploy(),
      new FenderHighShotComplete(),
      new FiveBallAutoPathPart1().deadlineWith(new IntakeAuto(1.0)),
      //new VisionHighShotComplete(),
      new DriveCameraAim().withTimeout(2),
      new ShooterVisionSetup().withTimeout(2),
      new Shoot().withTimeout(3),
      new FiveBallAutoPathPart2().deadlineWith(new IntakeAuto(1.0)),
      //new VisionHighShotComplete(),
      new DriveCameraAim().withTimeout(2),
      new ShooterVisionSetup().withTimeout(2),
      new Shoot().withTimeout(3)
    );
  }
}

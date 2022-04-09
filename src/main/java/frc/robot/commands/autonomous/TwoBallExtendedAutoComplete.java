// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveRotateToPosition;
import frc.robot.commands.intake.IntakeAuto;
import frc.robot.commands.intake.IntakeDeploy;
import frc.robot.commands.shooter.ShooterAngleSet;
import frc.robot.commands.shooter.ShooterKickerWheelSpinup;
import frc.robot.commands.shooter.ShooterWheelSpinup;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallExtendedAutoComplete extends SequentialCommandGroup {
  /** Creates a new TwoBallWithBallDefenceComplete. */
  public TwoBallExtendedAutoComplete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeDeploy().deadlineWith(new ShooterWheelSpinup(2000), new ShooterAngleSet(8), new ShooterKickerWheelSpinup(0.8)),
      new TwoBallExtendedAutoPathPart1().deadlineWith(new IntakeAuto(1.0)),
      new VisionHighShotComplete(),
      new TwoBallExtendedAutoPathPart2().deadlineWith(new IntakeAuto(1.0)),
      new DriveRotateToPosition(180),
      new InstantCommand(intakeSubsystem::retractIntake),
      new WaitCommand(0.5),
      new WaitCommand(1.0).deadlineWith(new IntakeAuto(-1.0))
    );
  }
}

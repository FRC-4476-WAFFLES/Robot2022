// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveCameraAim;
import frc.robot.commands.drive.DriveAuto.SwervePath;
import frc.robot.commands.intake.IntakeAuto;
import frc.robot.commands.intake.IntakeDeploy;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterAngleSet;
import frc.robot.commands.shooter.ShooterKickerWheelSpinup;
import frc.robot.commands.shooter.ShooterVisionSetup;
import frc.robot.commands.shooter.ShooterWheelSpinup;
import frc.robot.commands.utility.ProxyVariableCommand;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAutoCompleteVersion2 extends SequentialCommandGroup {
  /** Creates a new FiveBallAutoCompleteVersion2. */
  public FiveBallAutoCompleteVersion2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeDeploy().deadlineWith(new ShooterWheelSpinup(2200), new ShooterAngleSet(8), new ShooterKickerWheelSpinup(0.8)),
      new ResetToRightAutoStartingPosition(),
      new DriveAuto(
        new SwervePath(0, 0, -110, -110 + 180)
        .waypoint(-0.15, 1.025, -110)
        .finish(-0.3, 2.05, -88, -88 + 180, 3.0)
      ).deadlineWith(new IntakeAuto(1.0)),
      new IntakeAuto(1.0),
      new InstantCommand(intakeSubsystem::retractIntake),
      new VisionHighShotComplete(),
      new ProxyVariableCommand(() -> new DriveAuto(
        new SwervePath(-0.3, 2.05, driveSubsystem.getOdometryLocation().getRotation().getDegrees(), -88)
        .waypoint(-1.0, 1.3, -160)
        .waypoint(-2.30, 0.9, -160)
        .finish(-4.2, 0.7, -155, -155, 3.0)
      )).deadlineWith(new WaitCommand(0.5).andThen(intakeSubsystem::deployIntake).andThen(new IntakeAuto(1.0))),
      new VisionHighShotComplete(),
      new FiveBallAutoPathPart2().deadlineWith(new IntakeAuto(1.0)),
      new VisionHighShotSetup(),
      new Shoot().perpetually()
    );
  }
}

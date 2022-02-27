// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.ShooterConstants.Setup;
import frc.robot.commands.shooter.ShooterAngleSet;
import frc.robot.commands.shooter.ShooterKickerWheelSpinup;
import frc.robot.commands.shooter.ShooterWheelSpinup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShotSetup extends ParallelDeadlineGroup {
  /** Creates a new ShotSetup. */
  public ShotSetup(double targetRPM, double targetAngle, double targetKickerPower) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new ShooterWheelSpinup(targetRPM), new ShooterAngleSet(targetAngle), new ShooterKickerWheelSpinup(targetKickerPower));
    // addCommands(new FooCommand(), new BarCommand());
  }

  public ShotSetup(Setup targetSetup) {
    this(targetSetup.getTargetRPM(), targetSetup.getTargetAngle(), targetSetup.getTargetKickerPower());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.ShooterAngleSet;
import frc.robot.commands.shooter.ShooterKickerWheelSpinup;
import frc.robot.commands.shooter.ShooterWheelSpinup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FenderHighShotSetup extends ParallelCommandGroup {
  private boolean isActive = false;

  /** Creates a new FenderHighShotSetup. */
  public FenderHighShotSetup() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    addCommands(new ShooterWheelSpinup(2050), new ShooterAngleSet(0), new ShooterKickerWheelSpinup(0.8));
  }

  @Override
  public void execute() {
    super.execute();
    isActive = true;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    isActive = false;
  }

  /** Get a trigger that returns active when the command is running */
  public Trigger getIsActiveTrigger() {
    return new Trigger(() -> isActive);
  }
}

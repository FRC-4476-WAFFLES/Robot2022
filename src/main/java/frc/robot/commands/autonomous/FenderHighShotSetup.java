// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.commands.shooter.ShooterAngleSet;
//import frc.robot.commands.shooter.ShooterKickerWheelSpinup;
//import frc.robot.commands.shooter.ShooterWheelSpinup;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/*
public class FenderHighShotSetup extends ParallelCommandGroup {
  private boolean isActive = false;

  /** Creates a new FenderHighShotSetup. *//*
  public FenderHighShotSetup() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    addCommands(new ShooterWheelSpinup(2050), new ShooterAngleSet(0), new ShooterKickerWheelSpinup(0.8));
  }
/*
  @Override
  public void execute() {
    super.execute();
    isActive = true;
    System.err.println("Shooter Readying");
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    isActive = false;
  }*/

  /** Get a trigger that returns active when the command is running *//*
  public Trigger getIsActiveTrigger() {
    return new Trigger(() -> isActive);
  }
}
*/

public class FenderHighShotSetup extends CommandBase {
  private final double targetRPM = 2050;
  private boolean isActive = false;

  public FenderHighShotSetup() {
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeed(targetRPM);
    shooterSubsystem.setHoodAngle(0);
    shooterSubsystem.setKickerSpeed(0.8);
    isActive = true;
    System.err.println("Shooter Readying");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isActive = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.getShooterRPM() + ShooterConstants.RPMTolerance >= targetRPM 
    && Math.abs(shooterSubsystem.getHoodMotorPosition() - shooterSubsystem.getHoodMotorTargetPosition()) <= ShooterConstants.angleTolerance;
  }

  /** Get a trigger that returns active when the command is running */
  public Trigger getIsActiveTrigger() {
    return new Trigger(() -> isActive);
  }
}
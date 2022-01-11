// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleop extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  public DriveTeleop(DriveSubsystem driveSubsystem, Joystick leftJoystick, Joystick rightJoystick) {
    // Tell the scheduler that no other drive commands can be running while
    // this one is running.
    addRequirements(driveSubsystem);

    // Save driveSubsystem to a field so that it can be used by the other 
    // methods.
    this.driveSubsystem = driveSubsystem;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.robotDrive(leftJoystick.getY(), leftJoystick.getX(), rightJoystick.getX(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors when the command ends. If we didn't do this, they might
    // continue running during the next command.
    driveSubsystem.stop();
  }
}

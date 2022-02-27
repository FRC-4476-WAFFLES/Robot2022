// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class DriveCameraAim extends CommandBase {
  /** Creates a new DriveCameraAim. */
  public DriveCameraAim() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = Math.toRadians(vision.getHorizontal());
    driveSubsystem.robotDrive(0.0, 0.0, yaw * 8.0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

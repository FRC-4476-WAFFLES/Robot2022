// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class DriveTeleop extends CommandBase {

  public DriveTeleop() {
    // Tell the scheduler that no other drive commands can be running while
    // this one is running.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = leftJoystick.getY();
    double right = leftJoystick.getX();
    double rotation = rightJoystick.getX();

    SmartDashboard.putNumber("Left Joystick Y", forward);
    SmartDashboard.putNumber("Left Joystick X", right);
    SmartDashboard.putNumber("Right Joystick X", rotation);

    if (Math.abs(forward) < .05){
      forward = 0;
    }
    if(Math.abs(right) < .05){
      right = 0;
    }
    if(Math.abs(rotation) < .05){
      rotation = 0;
    }

    forward *= 4;
    right *= 4;
    rotation *= 6;
    
    driveSubsystem.robotDrive(forward, right, rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors when the command ends. If we didn't do this, they might
    // continue running during the next command.
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

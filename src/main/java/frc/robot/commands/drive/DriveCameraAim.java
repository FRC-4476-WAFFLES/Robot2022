// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

public class DriveCameraAim extends CommandBase {
  PIDController turnController = new PIDController(-4.0, 0, -0.2);

  /** Creates a new DriveCameraAim. */
  public DriveCameraAim() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.err.println("Drive aim supposedly commencing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.err.println("Drive aim supposedly executing");
    vision.setLEDMode(CameraLEDMode.On);
    turnController.setSetpoint(0);
    double turnSpeed = turnController.calculate(Math.toRadians(vision.getFilteredHorizontal()));
    driveSubsystem.robotDrive(0.0, 0.0, turnSpeed, true);
    SmartDashboard.putNumber("Drive Current Rotation Error (Radians)", Math.toRadians(vision.getFilteredHorizontal()));
    SmartDashboard.putNumber("Drive PID Turn Speed", turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.err.println("Drive aim supposedly finished");
    driveSubsystem.stop();
    vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(vision.getHorizontal()) < 2.0 && vision.getHasTarget();
  }
}

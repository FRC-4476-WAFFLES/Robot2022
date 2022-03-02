// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera.CameraLEDMode;

import static frc.robot.RobotContainer.*;

public class DriveCameraAim extends CommandBase {
  private final double pCoefficient = 8.0;
  private final double dCoefficient = 1.0;

  private double previousYawError = 0;
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
    vision.setLEDMode(CameraLEDMode.On);
    double yawError = Math.toRadians(vision.getFilteredHorizontal());
    double pValue = yawError * pCoefficient;
    double dValue = (yawError - previousYawError) * dCoefficient * 0.02;
    double pdControlledVelocity = pValue + dValue;
    driveSubsystem.robotDrive(0.0, 0.0, pdControlledVelocity, true);
    previousYawError = yawError;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    vision.setLEDMode(CameraLEDMode.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

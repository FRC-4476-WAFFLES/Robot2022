// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class ClimberAnalogStickControl extends CommandBase {
  /** Creates a new ClimberAnalogStickControl. */
  public ClimberAnalogStickControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double movement = -operate.getLeftY();
    if (Math.abs(movement) < 0.05) {
      movement = 0;
    }
    climberSubsystem.moveClimberWithAnalogStick(movement);

    double otherMovement = -operate.getRightY();
    if (Math.abs(otherMovement) < 0.05) {
      otherMovement = 0;
    }
    climberSubsystem.moveCLimberPivotWithAnalogStick(otherMovement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

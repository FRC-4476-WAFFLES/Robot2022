// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.ClimberPositions;

import static frc.robot.RobotContainer.*;

public class ClimberSetPosition extends CommandBase {
  private ClimberPositions target;
  /** Creates a new SetPosition. */
  public ClimberSetPosition(ClimberPositions target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.setTarget(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

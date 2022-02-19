// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class IntakePowerRun extends CommandBase {
  /** Creates a new IntakePowerRun. */
  public IntakePowerRun() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -operate.getLeftTriggerAxis() + operate.getRightTriggerAxis();
    if (conveyorSubsystem.shouldRun()) {
      conveyorSubsystem.runConveyor(Math.min(power, 0.5));
    } else {
      conveyorSubsystem.stopConveyor();
    }
    intakeSubsystem.runIntake(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    conveyorSubsystem.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

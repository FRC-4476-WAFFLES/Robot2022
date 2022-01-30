// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class Intake2Balls extends CommandBase {
  /** Creates a new IntakeIn. */
  public Intake2Balls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.shouldRun()) {
        intakeSubsystem.runIntakeIn();
    } else {
        intakeSubsystem.stopIntake();
    }

    if (intakeSubsystem.shouldConveyorRun()) {
        conveyorSubsystem.runConveyorIn();
    } else {
        conveyorSubsystem.stopConveyor();
    }
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
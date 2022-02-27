// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class IntakeAuto extends CommandBase {
  private final double power;
  /** Creates a new IntakeAuto. */
  public IntakeAuto(double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.power = power;
    addRequirements(intakeSubsystem, conveyorSubsystem);
  }

  public IntakeAuto() {
    this(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (conveyorSubsystem.shouldRun()) {
      conveyorSubsystem.runConveyor(Math.min(power, 0.3));
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class Shoot extends CommandBase {
  /** Creates a new ShootAllBalls. */
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intakeSubsystem.runIntake(1.0);
    SmartDashboard.putString("Conveyor", "Is being told to run");
    conveyorSubsystem.runConveyor(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intakeSubsystem.stopIntake();
    conveyorSubsystem.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !conveyorSubsystem.getLowIR() && !conveyorSubsystem.getLowIRPreviousState() && !conveyorSubsystem.getMidIR() && !conveyorSubsystem.getHighIR();
    //return false;
  }
}

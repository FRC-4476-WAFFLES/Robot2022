// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class IntakeTeleop extends CommandBase {
  /** Creates a new IntakePowerRun. */
  public IntakeTeleop() {
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
    if (power >= 0.01 && !conveyorSubsystem.getHighIR()) {
      // for testing
      // conveyorSubsystem.runConveyor(Math.min(power, 0.2)); 
      // intakeSubsystem.runIntake(power);
      // if (Math.abs(power) >= 0.01) {
      //   intakeSubsystem.deployIntake();
      //   conveyorSubsystem.runConveyor(Math.min(power, 0.2));
      //   intakeSubsystem.runIntake(power);
      // }else{
      //   intakeSubsystem.retractIntake();
      //   intakeSubsystem.runIntake(0.0);
      //   conveyorSubsystem.runConveyor(0.0);
      // }
      
      intakeSubsystem.deployIntake();
    
      if (conveyorSubsystem.shouldRun()) {
        conveyorSubsystem.runConveyor(Math.min(power, 0.2));
      } else {
        conveyorSubsystem.stopConveyor();
      }
      
      intakeSubsystem.runIntake(power);
      shooterSubsystem.setKickerSpeed(0.0);

    } else if (power <= 0.01) {
      intakeSubsystem.runIntake(0.0);
      conveyorSubsystem.runConveyor(power);
      intakeSubsystem.retractIntake();
      shooterSubsystem.setKickerSpeed(power);

    } else {
      intakeSubsystem.retractIntake();
      intakeSubsystem.runIntake(0.0);
      conveyorSubsystem.runConveyor(0.0);
      shooterSubsystem.setKickerSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    conveyorSubsystem.stopConveyor();
    shooterSubsystem.setKickerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
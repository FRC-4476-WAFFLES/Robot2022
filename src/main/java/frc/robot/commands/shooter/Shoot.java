// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class Shoot extends CommandBase {
  private Timer timeSinceLastHasTarget = new Timer();
  private Timer startTimer = new Timer();
  private double savedDistanceToGoal = 0; // Set the savedLimelightTY to be a value to get the hood to go to fender shot angle
  
  /** Creates a new ShootAllBalls. */
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {/*
    timeSinceLastHasTarget.reset();
    timeSinceLastHasTarget.start();
    startTimer.reset();
    startTimer.start();*/
    SmartDashboard.setDefaultNumber("Set Conveyor Shoot Speed", 0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intakeSubsystem.runIntake(1.0);
    //SmartDashboard.putString("Conveyor", "Is being told to run");
    //conveyorSubsystem.runConveyor(0.3);

    double conveyorTargetSpeed;

    if (vision.getHasTarget()) {
      //System.err.println("Shooter has target");
      timeSinceLastHasTarget.reset();
      double distanceToGoal = vision.getHorisontalFieldDistanceToGoal();
      savedDistanceToGoal = distanceToGoal;
      //conveyorTargetSpeed = calculateConveyorPower(distanceToGoal);
      conveyorTargetSpeed = 0.55;

    } else if (timeSinceLastHasTarget.get() < 0.5) {
      if (startTimer.get() < 0.5) {
        //System.err.println("1");
        conveyorTargetSpeed = 0.3;

      } else {
        //System.err.println("2");
        //conveyorTargetSpeed = calculateConveyorPower(savedDistanceToGoal);
        conveyorTargetSpeed = 0.55;
      }

    } else {
      //System.err.println("3");
      conveyorTargetSpeed = 0.3;
    }

    conveyorSubsystem.runConveyor(conveyorTargetSpeed);
    /*
    double conveyorTargetSpeed = SmartDashboard.getNumber("Set Conveyor Shoot Speed", 0);
    conveyorSubsystem.runConveyor(conveyorTargetSpeed);*/
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

  private double calculateConveyorPower(double distance) {
    return 0.133 * distance + 0.426;
  }
}

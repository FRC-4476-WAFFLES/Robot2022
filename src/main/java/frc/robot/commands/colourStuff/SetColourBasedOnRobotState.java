// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.colourStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightController.LightColours;

import static frc.robot.RobotContainer.*;

public class SetColourBasedOnRobotState extends CommandBase {
  /** Creates a new SetColourBasedOnRobotState. */
  public SetColourBasedOnRobotState() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getIsReady()) {
      // System.err.println("Blinking because shooter is ready");
      lightController.blinkBetweenColours(LightColours.PINK, LightColours.BLACK);
    } else if (vision.getHasTarget()) {
      lightController.setLightColour(LightColours.PINK);
    } else if (conveyorSubsystem.getHighIR()) {
      lightController.setLightColour(LightColours.GREEN);
    } else if (conveyorSubsystem.getLowIR() || conveyorSubsystem.getLowIRPreviousState() || conveyorSubsystem.getMidIR()) {
      lightController.setLightColour(LightColours.BLUE);
    } else {
      lightController.setLightColour(LightColours.YELLOW);
    }
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

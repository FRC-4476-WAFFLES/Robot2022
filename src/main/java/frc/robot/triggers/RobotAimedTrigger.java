// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;

import static frc.robot.RobotContainer.*;

/** Add your docs here. */
public class RobotAimedTrigger extends Trigger {
  @Override
  public boolean get() {
    //return false;
    return vision.getHasTarget() && Math.abs(vision.getHorizontal()) < SwerveConstants.aimToleranceDegrees;
  }
}

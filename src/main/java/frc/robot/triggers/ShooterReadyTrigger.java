package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;

import static frc.robot.RobotContainer.*;

public class ShooterReadyTrigger extends Trigger {
    @Override
    public boolean get() {
        //return shooterSubsystem.getFilteredShooterRPM() + ShooterConstants.RPMTolerance >= shooterSubsystem.getShooterTargetRPM(); // Return an active trigger if the shooter is at its desired speed
        return Math.abs(shooterSubsystem.getShooterRPM() - shooterSubsystem.getShooterTargetRPM()) <= ShooterConstants.RPMTolerance 
        && Math.abs(shooterSubsystem.getHoodMotorPosition() - shooterSubsystem.getHoodMotorTargetPosition()) <= ShooterConstants.angleTolerance;
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeRoller = new TalonSRX(Constants.INTAKE_ROLLER);

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    
  }

  @Override
  public void periodic() {

  }
}
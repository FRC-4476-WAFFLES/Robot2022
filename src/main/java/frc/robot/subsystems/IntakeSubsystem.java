package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeConstants intakeConstants = new IntakeConstants();

    private final TalonSRX intakeRoller = new TalonSRX(intakeConstants.INTAKE_ROLLER);
    private final CANSparkMax leftDeployIntakeMotor = new CANSparkMax(intakeConstants.INTAKE_DEPLOY_LEFT, MotorType.kBrushless);
    private final CANSparkMax rightDeployIntakeMotor = new CANSparkMax(intakeConstants.INTAKE_DEPLOY_RIGHT, MotorType.kBrushless);

    private final DigitalInput firstIR = new DigitalInput(intakeConstants.firstIR);
    private final DigitalInput secondIR = new DigitalInput(intakeConstants.secondIR);
    private final DigitalInput thirdIR = new DigitalInput(intakeConstants.thirdIR);

    /**
     * Creates a new IntakeSubsystem
     */
    public IntakeSubsystem() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Current", intakeRoller.getSupplyCurrent());
        
    }

    public void intake(final double percent) {
        // TODO: intake implementation


        intakeRoller.set(ControlMode.PercentOutput, percent);
    }

    public void outtake(final double percent, boolean withConveyor) {
        // TODO: outtake implementation

    }
}

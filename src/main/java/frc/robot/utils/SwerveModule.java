// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    /** Holds constants like the angle calibration. */
    private final SwerveConstants constants;

    /** The motor controlling the angle of the swerve module. */
    private final TalonFX angleMotor;

    /** The motor controlling the speed of the swerve module. */
    private final TalonFX driveMotor;

    public SwerveModule(SwerveConstants constants) {
        this.angleMotor = new TalonFX(constants.angleMotor);
        this.driveMotor = new TalonFX(constants.driveMotor);
        this.constants = constants;

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.03));
        angleMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.03));

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
        angleMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
        driveMotor.configVelocityMeasurementWindow(4);
        angleMotor.configVelocityMeasurementWindow(4);

        driveMotor.configVoltageCompSaturation(12);
        angleMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);
        angleMotor.enableVoltageCompensation(true);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.config_kP(0, 0.2);
        angleMotor.config_kI(0, 0);
        angleMotor.config_kD(0, 0.1);
    }

    /** Drives the swerve module in a direction at a speed.
     * 
     * @param desired Desired state of the swerve module
     */
    public void drive(SwerveModuleState desired) {
        double currentAngleRaw = angleMotor.getSelectedSensorPosition() / constants.steeringDegreesToTicks;
        //double currentAngleVelocityRaw = angleMotor.getSelectedSensorVelocity();

        double currentAngle = currentAngleRaw % 360;
        if (currentAngle < -180) {
            currentAngle += 360;
        } else if (currentAngle > 180) {
            currentAngle -= 360;
        }

        //double velocityOffset = currentAngleVelocityRaw * constants.steeringThingy;

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(currentAngle));
        
        // The minus function will currently give you an angle from -180 to 180.
        // If future library versions change this, this code will no longer work.
        double targetAngle = currentAngleRaw + optimizedState.angle.minus(Rotation2d.fromDegrees(currentAngleRaw)).getDegrees();

        angleMotor.set(ControlMode.Position, targetAngle * constants.steeringDegreesToTicks);
        //driveMotor.set(ControlMode.Velocity, optimizedState.speedMetersPerSecond * constants.metersPerSecondToTicksPer100ms);
        driveMotor.set(ControlMode.PercentOutput, optimizedState.speedMetersPerSecond);
    }

    /**
     * Get the current state of a swerve module
     * @return SwerveModuleState representing the current speed and angle of the module
     */
    
    public SwerveModuleState getState(){
        double currentAngle = (angleMotor.getSelectedSensorPosition() / constants.steeringDegreesToTicks) % 360;
        if (currentAngle < -180) {
            currentAngle += 360;
        } else if (currentAngle > 180) {
            currentAngle -= 360;
        }
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity() / constants.metersPerSecondToTicksPer100ms, 
            Rotation2d.fromDegrees(currentAngle));
    }

    /** Stops all motors from running. */
    public void stop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
        this.angleMotor.set(ControlMode.PercentOutput, 0);
    }
}

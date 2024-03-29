// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // TODO: set ports of all motors and actuators
    // PWM
    public static final int lightsBlinkin = 0; // Blinkin for controlling aRGB lights

    // CAN bus
    public static final int swerveModule1Angle = 6; // FX Front Left 2
    public static final int swerveModule1Drive = 7; // FX Front Left 1
    public static final int swerveModule2Angle = 8; // FX Back Left 2
    public static final int swerveModule2Drive = 9; // FX Back Left 1
    public static final int swerveModule3Angle = 2; // FX Back Right 1
    public static final int swerveModule3Drive = 3; // FX Back Right 2
    public static final int swerveModule4Angle = 4; // FX Front Right 1
    public static final int swerveModule4Drive = 5; // FX Front Right 2
    public static final int shooterSpinRight = 14; // FX
    public static final int shooterSpinLeft = 17; // FX
    public static final int intakeSpin = 13; // SRX
    public static final int intakeDeployLeft = 10; // Spark Max
    public static final int intakeDeployRight = 11; // Spark Max
    public static final int conveyorSpin = 12; // Spark Max
    public static final int kickerWheelSpin = 18; // SRX
    public static final int climbLeft = 16; // FX
    public static final int climbRight = 15; // FX
    public static final int climbPivotLeft = 19; // FX
    public static final int climbPivotRight = 20; // FX
    public static final int shooterAngle = 60; // Spark Max

    // I2C Bus
    public static final int MPUAddress = 0x68;

    // SPI Bus
    public static final SPI.Port gyroPort = SPI.Port.kOnboardCS0;

    // Digital Input
    //public static final int intakeLimit = 0; //  am-3313 Hall Effect Sensor
    
    public static final int swerveModule1Encoder = 0; // SRX Mag Encoder
    public static final int swerveModule2Encoder = 1; // SRX Mag Encoder
    public static final int swerveModule3Encoder = 2; // SRX Mag Encoder
    public static final int swerveModule4Encoder = 3; // SRX Mag Encoder
    public static final int highIR = 4; // IR
    public static final int midIR = 5; // IR
    public static final int lowIR = 6; // IR

    public static final class SwerveConstants {
        /** Represents the offset from the centre of the robot, in metres. */
        public final Translation2d position;

        /**
         * Stores the angle offset of this particular swerve module, in degrees. This
         * will be used to compensate for the different "zero" angles of the encoders.
         * 
         * To calibrate this value, manually rotate each module to be facing the same
         * direction. When they are all aligned,
         */
        public final double calibration;

        /** The CAN address of the module's angle motor. */
        public final int angleMotor;
        /** The CAN address of the module's drive motor. */
        public final int driveMotor;

        public final int angleEncoder;

        public final double CPR = 2048; // Encoder ticks per motor rotation
        public final double wheelDiameter = 0.1016; // Wheel diameter in meters
        public final double wheelCircumfrence = wheelDiameter * Math.PI; // Wheel circumfrence in meters

        public final double firstStageRatio = 14.0/50.0;
        public final double secondStageRatio = 28.0/16.0;
        public final double thirdStageRatio = 15.0/60.0;
        public final double driveOverallRatio = 1.0 / (firstStageRatio * secondStageRatio * thirdStageRatio); // Drive gear ratio

        public final double steeringRatio = 12.8;

        public final double steeringToDriveRatio = 1.0 / (firstStageRatio * secondStageRatio * steeringRatio);
        
        public final double metersPerSecondToTicksPer100ms = CPR * driveOverallRatio / wheelCircumfrence / 10.0;

        public static final double maxAttainableSpeedMetersPerSecond = 4.1;
        public static final double maxAttainableRotationRateRadiansPerSecond = 8.0;
        public static final double maxAccelerationMetersPerSecondSquared = 2.9;

        // The number of ticks of the motor's built-in encoder per revolution of the steering module
        public final double ticksPerSteeringRevolution = 26214.4;
        // Convert degrees to motor ticks
        public final double steeringDegreesToTicks = ticksPerSteeringRevolution / 360.0;

        public SwerveConstants(Translation2d position, double calibration, int angleMotor, int driveMotor, int angleEncoder) {
            this.position = position;
            this.calibration = calibration;
            this.angleMotor = angleMotor;
            this.driveMotor = driveMotor;
            this.angleEncoder = angleEncoder;
        }
    }

    public static final SwerveConstants swerveModules[] = new SwerveConstants[] {
        // Modules are in the order of Front Left, Back Left, Back Right, Front Right, when intake is front of robot
        new SwerveConstants(new Translation2d(0.2921, 0.2921), 48.68, swerveModule1Angle, swerveModule1Drive, swerveModule1Encoder),
        new SwerveConstants(new Translation2d(-0.2921, 0.2921), 229.16, swerveModule2Angle, swerveModule2Drive, swerveModule2Encoder),
        new SwerveConstants(new Translation2d(-0.2921, -0.2921), 230.14, swerveModule3Angle, swerveModule3Drive, swerveModule3Encoder),
        new SwerveConstants(new Translation2d(0.2921, -0.2921), 267.42, swerveModule4Angle, swerveModule4Drive, swerveModule4Encoder),
    };

    public static final class ClimberConstants {
        public final double leftRotatingSetpoints;
        public final double leftExtendingSetpoints;
        public final double rightRotatingSetpoints;
        public final double rightExtendingSetpoints;
        
        public ClimberConstants(double leftRotatingSetpoints, double leftExtendingSetpoints, double rightExtendingSetpoints, double rightRotatingSetpoints) {
            this.leftRotatingSetpoints = leftRotatingSetpoints;
            this.leftExtendingSetpoints = leftExtendingSetpoints;
            this.rightRotatingSetpoints = rightRotatingSetpoints;
            this.rightExtendingSetpoints = rightExtendingSetpoints;
        }
    }

    public static final class ShooterConstants {
        public static final double shooterkP = 0.05;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.0;
        public static final double shooterkF = 0.0505; //0.053
        // public final double kF = 1023.0/20660.0; // kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
        // 1023.0/20660.0 = 0.04951597
        
        public static final double anglekP = 20.0;
        public static final double anglekI = 0.0;
        public static final double anglekD = 0.0;
        public static final double anglekF = 0.0;

        public static final double RPMTolerance = 75;
        public static final double angleTolerance = 0.25;

        //TODO: change following variables to reflect actual design
        public static final double minAngle = 10;
        public static final double maxAngle = 30;

        public static final double stage1AngleRatio = 12/24;
        public static final double stage2AngleRatio = 20/432;
        public static final double motorRotationsToHoodDegreesMoved = 1 / (stage1AngleRatio * stage2AngleRatio * 360);
    }
}

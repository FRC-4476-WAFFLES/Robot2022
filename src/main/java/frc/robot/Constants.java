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
    public static final int shooterHoodAngle1 = 0; // am-3517 Linear Servo
    public static final int shooterHoodAngle2 = 1; // am-3517 Linear Servo
    // See https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMTkvMDMvMjIvMTAvMjcvNTgvMDMxOTQ4ODUtYmM5Yi00M2UyLWE1NDAtZGNiMWVhNzEzMDEzL1VzaW5nIEwxNiBMaW5lYXIgU2Vydm8gMDMtMjAxOS5wZGYiXV0/Using%20L16%20Linear%20Servo%2003-2019.pdf?sha=ee4c9607408cc835
    // for information on calibrating the linear servos

    // CAN bus
    public static final int swerveModule1Angle = 1; // FX
    public static final int swerveModule1Drive = 2; // FX
    public static final int swerveModule2Angle = 3; // FX
    public static final int swerveModule2Drive = 4; // FX
    public static final int swerveModule3Angle = 5; // FX
    public static final int swerveModule3Drive = 6; // FX
    public static final int swerveModule4Angle = 7; // FX
    public static final int swerveModule4Drive = 8; // FX
    public static final int shooterSpin1 = 9; // FX
    public static final int shooterSpin2 = 10; // FX
    public static final int intakeSpin = 11; // SRX
    public static final int intakeDeploy1 = 12; // Spark Max
    public static final int intakeDeploy2 = 13; // Spark Max
    public static final int conveyorSpin = 14; // SRX
    public static final int kickerWheelSpin = 15; // SRX
    public static final int climbLeft = 16; // FX
    public static final int climbRight = 17; // FX
    public static final int climbPivotLeft = 18; // FX
    public static final int climbPivotRight = 19; // FX

    // I2C Bus
    public static final int MPUAddress = 0x68;

    // SPI Bus
    public static final SPI.Port gyroPort = SPI.Port.kOnboardCS0;

    // Digital Input
    public static final int intakeLimit = 0; //  am-3313 Hall Effect Sensor
    public static final int lowIR = 1; // IR
    public static final int midIR = 2; // IR
    public static final int highIR = 3; // IR
    public static final int swerveModule1Encoder = 4; // SRX Mag Encoder
    public static final int swerveModule2Encoder = 5; // SRX Mag Encoder
    public static final int swerveModule3Encoder = 6; // SRX Mag Encoder
    public static final int swerveModule4Encoder = 7; // SRX Mag Encoder

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
        /** The DIO address of the module's angle encoder. */
        public final int angleEncoder;

        public final double CPR = 2048; // Encoder ticks per motor rotation
        public final double wheelDiameter = 0.1016; // Wheel diameter in meters
        public final double wheelCircumfrence = wheelDiameter * Math.PI; // Wheel circumfrence in meters

        public final double firstStageRatio = 14/50;
        public final double secondStageRatio = 28/16;
        public final double thirdStageRatio = 15/60;
        public final double driveOverallRatio = 1 / (firstStageRatio * secondStageRatio * thirdStageRatio); // Drive gear ratio

        public final double steeringRatio = 12.8;

        public final double steeringThingy = 1 / (firstStageRatio * secondStageRatio * steeringRatio); // TODO: name this something more understandable
        
        public final double metersPerSecondToTicksPer100ms = CPR * driveOverallRatio / wheelCircumfrence / 10;

        public static final double maxAttainableSpeedMetersPerSecond = 4;

        // The number of ticks of the motor's built-in encoder per revolution of the steering module
        public final double ticksPerSteeringRevolution = 26214.4;
        // Convert degrees to motor ticks
        public final double steeringDegreesToTicks = ticksPerSteeringRevolution / 360;

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
        new SwerveConstants(new Translation2d(0.2921, 0.2921), 0, swerveModule1Angle, swerveModule1Drive, swerveModule1Encoder),
        new SwerveConstants(new Translation2d(-0.2921, 0.2921), 0, swerveModule2Angle, swerveModule2Drive, swerveModule2Encoder),
        new SwerveConstants(new Translation2d(-0.2921, -0.2921), 0, swerveModule3Angle, swerveModule3Drive, swerveModule3Encoder),
        new SwerveConstants(new Translation2d(0.2921, -0.2921), 0, swerveModule4Angle, swerveModule4Drive, swerveModule4Encoder),
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
        public final double kP = 0.5;
        public final double kI = 0.001;
        public final double kD = 0.1;
        public final double kF = 1023.0/20660.0; // kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
        public final int kIzone = 300;
        public final double kPeakOutput = 1.00;
    }
}

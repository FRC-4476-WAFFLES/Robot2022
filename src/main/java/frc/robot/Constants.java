// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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

        public SwerveConstants(Translation2d position, double calibration, int angleMotor, int driveMotor) {
            this.position = position;
            this.calibration = calibration;
            this.angleMotor = angleMotor;
            this.driveMotor = driveMotor;
        }
    }

    public static final class MPUConstants{
        public final double calibration = 0;
        public final int address = 0x68;
        public MPUConstants(){}
    }

    public static final SwerveConstants swerveModules[] = new SwerveConstants[] {
        // TODO: set CAN ports of motors
        new SwerveConstants(new Translation2d(-0.1715, -0.4191), 0, 1, 2),
        new SwerveConstants(new Translation2d(0.1715, -0.4191), 0, 3, 4),
        new SwerveConstants(new Translation2d(0.1715, 0.4191), 0, 5, 6),
        new SwerveConstants(new Translation2d(-0.1715, 0.4191), 0, 7, 8),
    };

    public static final class IntakeConstants {
        // TODO: set CAN ports of intake motor and motor controllers
        public final int INTAKE_ROLLER = 0;
        public final int INTAKE_DEPLOY_LEFT = 0;
        public final int INTAKE_DEPLOY_RIGHT = 0;

        public final int firstIR = 0;
        public final int secondIR = 0;
        public final int thirdIR = 0;

        public IntakeConstants(){}
    }
}

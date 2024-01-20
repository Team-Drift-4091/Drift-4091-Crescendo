// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Units are in meters and radians
 */
public class Constants {
    public static final double FIELD_WIDTH = Units.feetToMeters(27);
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static class DrivetrainConstants {
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
        public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 8.14; // MK4i L1
        // Dimensions
        public static final double WHEEL_BASE_LENGTH_METERS = Units.inchesToMeters(24 - (2.625 * 2));
        public static final double WHEEL_BASE_WIDTH_METERS = WHEEL_BASE_LENGTH_METERS; // It's a square

        public static final double rotationalP = 4;
        public static final double rotationalI = 0.7;
        public static final double rotationalD = 0.6;

        public static final double rotationalPForHolonomic = 4.5;
        public static final double rotationalIForHolonomic = 0;
        public static final double rotationalDForHolonomic = 0;

        public static final double translationalP = 2.5;
        public static final double translationalI = 0.5;
        public static final double translationalD = 0.4;

        public static final double CHASSIS_MAX_VELOCITY = 3.;
        public static final double CHASSIS_MAX_ACCELERATION = 1.5;
        public static final double CHASSIS_MAX_ANGULAR_VELOCITY = Math.PI;
        public static final double CHASSIS_MAX_ANGULAR_ACCELERATION = Math.PI;
        public static final double MODULE_MAX_VELOCITY = 3.75; // Free speed max is ~3.96 for MK4i with L1

        public static final double driveP = 0.02;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveF = 0.045;

        public static final double steerP = 0.9;
        public static final double steerI = 0.0;
        public static final double steerD = 0.0;
        public static final double steerF = 0.0;

        public static class FrontLeft {
            public static final int DRIVE_CHANNEL = 1;
            public static final int STEER_CHANNEL = 2;
            public static final int CANCODER_CHANNEL = 9;
            public static final double ENCODER_OFFSET_DEGREES = 126.826172;
        }
        public static class FrontRight {
            public static final int DRIVE_CHANNEL = 3;
            public static final int STEER_CHANNEL = 4;
            public static final int CANCODER_CHANNEL = 10;
            public static final double ENCODER_OFFSET_DEGREES = 150.556641;
        }
        public static class BackLeft {
            public static final int DRIVE_CHANNEL = 5;
            public static final int STEER_CHANNEL = 6;
            public static final int CANCODER_CHANNEL = 11;
            public static final double ENCODER_OFFSET_DEGREES = 148.095703+180;
        }
        public static class BackRight {
            public static final int DRIVE_CHANNEL = 7;
            public static final int STEER_CHANNEL = 8;
            public static final int CANCODER_CHANNEL = 12;
            public static final double ENCODER_OFFSET_DEGREES = 228.955078;
        }
    }
    public static class VisionConstants {
        public static final String CAMERA_NAME = "Limelight";
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(-0.1778, 0.3302, 0), // in meters
            new Rotation3d(0, 0, Math.PI)
        );
    }
    public static class ClawConstants {
        public static final int DIGITAL_INPUT_CHANNEL = 1;
        public static final int INNER_LEFT_MOTOR_ID = 14;
        public static final int INNER_RIGHT_MOTOR_ID = 15;
        public static final int OUTER_LEFT_MOTOR_ID = 16;
        public static final int OUTER_RIGHT_MOTOR_ID = 17;
    }
    public static class ClawJointConstants {
        public static final int MOTOR_ID = 13;
        public static final double MIN_ANGLE = .975; // .95 rotations
        public static final double MAX_ANGLE = .314; //.314 rotations

        // https://docs.revrobotics.com/through-bore-encoder/specifications
        public static final int ENCODER_PULSES_PER_ROTATION = 1024;
    }
}
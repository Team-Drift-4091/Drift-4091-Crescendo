// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.utility.MirrorPoses.mirror;

// Helper methods that return a target Pose2d to align to a hybrid or cube node.
// Also contains an enum holding the shooting motor speeds for each scoring level.
public class ScoringLocationManager {
    private ScoringLocationManager() {}

    public enum ScoringLevel {
        HYBRID(.12),
        MID(.2),
        HIGH(.25);

        private final double motorSpeed;
        ScoringLevel(double motorSpeed) {
            this.motorSpeed = motorSpeed;
        }

        public double getRequiredMotorSpeed() {
            return motorSpeed;
        }
    }

    private static final double DISTANCE_FROM_WALL = 2;

    public static Pose2d getNearestNodeAlignment() {
        final double currentPoseY = Drivetrain.getInstance().getPose2d().getY();
        final double nodeSpacing = Units.inchesToMeters(22);
        final double firstNodePosition = Units.inchesToMeters(20);
        final double offset = firstNodePosition - (nodeSpacing / 2.);

        final int nodeNumber = (int) ((currentPoseY - offset) / nodeSpacing);
        // Return if the robot's pose is too high
        if (nodeNumber > 8) {
            return Drivetrain.getInstance().getPose2d();
        }
        
        final double nearestPoseY = nodeNumber * nodeSpacing + (nodeSpacing / 2.) + offset;
        if (Robot.isRedAlliance()) {
            return mirror(new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, new Rotation2d()));
        }
        return new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, new Rotation2d());
    }

    public static Pose2d getNearestCubeNodeAlignment() {
        final double currentPoseY = Drivetrain.getInstance().getPose2d().getY();
        final double nodeSpacing = Units.inchesToMeters(66);
        final double firstNodePosition = Units.inchesToMeters(42);
        final double offset = firstNodePosition - (nodeSpacing / 2.);

        final int nodeNumber = (int) ((currentPoseY - offset) / nodeSpacing);
        // Return if the robot's pose is too high
        if (nodeNumber > 2) {
            return Drivetrain.getInstance().getPose2d();
        }
        
        final double nearestPoseY = nodeNumber * nodeSpacing + (nodeSpacing / 2.) + offset;
        if (Robot.isRedAlliance()) {
            return mirror(new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, new Rotation2d()));
        }
        return new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, new Rotation2d());
    }
}

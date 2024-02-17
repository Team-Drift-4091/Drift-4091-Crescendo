// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.FIELD_LENGTH;

/**
 * This year the field isn't rotationally symetrical, so we had to reflect robot poses when on red alliance since our odometry 
 * is relative to the april tag origin, on the side of the blue alliance driver stations.
 * 
 * Any poses or rotations given to the robot as target positions to drive to should be run through one of these methods first.
 * 
 * Set poses and rotations assuming the robot is on the blue alliance.
 * 
 * Example:
 * Pair<Pose2d, Rotation2d> poseAndRotation = MirrorPoses.pairFromDegrees(4.0, 3.5, 180, 45);
*/
public class MirrorPoses {
    private MirrorPoses() {}

 

    /**
     * If the robot is on the red alliance, the pose and rotation will be mirrored as if the robot were on the other alliance
     * @param poseAndHeading the {@link Pair}<{@link Pose2d},{@link Rotation2d}> that will be mirrored
     * @return the pose and heading that has been mirrored if necessary
     */
    public static Pair<Pose2d, Rotation2d> mirror(Pair<Pose2d, Rotation2d> poseAndHeading) {
        return new Pair<Pose2d,Rotation2d>(
            mirror(poseAndHeading.getFirst()), 
            mirror(poseAndHeading.getSecond())
        );
    }

    /**
     * If the robot is on the red alliance, the pose will be mirrored as if the robot were on the other alliance
     * @param pose the {@link Pose2d} that will be mirrored
     * @return the pose that has been mirrored if necessary
     */
    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), mirror(pose.getRotation()));
    }

    /**
     * If the robot is on the red alliance, the pose will be mirrored as if the robot were on the other alliance
     * @param translation the {@link Translation2d} that will be mirrored
     * @return the pose that has been mirrored if necessary
     */
    public static Translation2d mirror(Translation2d translation) {
        return new Translation2d(FIELD_LENGTH - translation.getX(), translation.getY());
    }

    /**
     * If the robot is on the red alliance, the rotation will be mirrored as if the robot were on the other alliance
     * @param pose the {@link Rotation2d} that will be mirrored
     * @return the rotation that has been mirrored if necessary
     */
    public static Rotation2d mirror(Rotation2d rotation) {
        return Rotation2d.fromDegrees(180).minus(rotation);
    }

    /**
     * Creates a {@link Pair}<{@link Pose2d}, {@link Rotation2d}> that is mirrored if the robot is on the red alliance.
     * @param x pose x
     * @param y pose y
     * @param poseRotation2d pose rotation
     * @param headingRotation2d heading rotation
     * @return the pose and heading that is mirrored if necessary
     */
    public static Pair<Pose2d, Rotation2d> pairFromRotation2d(double x, double y, Rotation2d poseRotation2d, Rotation2d headingRotation2d) {
        return mirror(new Pair<Pose2d,Rotation2d>(new Pose2d(x, y, poseRotation2d), headingRotation2d));
    }

    /**
     * Creates a {@link Pair}<{@link Pose2d}, {@link Rotation2d}> that is mirrored if the robot is on the red alliance.
     * @param x pose x
     * @param y pose y
     * @param poseDegrees pose rotation
     * @param headingDegrees heading rotation
     * @return the pose and heading that is mirrorred if necessary
     */
    public static Pair<Pose2d, Rotation2d> pairFromDegrees(double x, double y, double poseDegrees, double headingDegrees) {
        return mirror(new Pair<Pose2d,Rotation2d>(new Pose2d(x, y, Rotation2d.fromDegrees(poseDegrees)), Rotation2d.fromDegrees(headingDegrees)));
    }

    /**
     * Creates a {@link Pair}<{@link Pose2d}, {@link Rotation2d}> that is mirrored if the robot is on the red alliance.
     * @param x pose x
     * @param y pose y
     * @param poseRadians pose rotation
     * @param headingRadians heading rotation
     * @return the pose and heading that is mirrorred if necessary
     */
    public static Pair<Pose2d, Rotation2d> pairFromRadians(double x, double y, double poseRadians, double headingRadians) {
        return mirror(new Pair<Pose2d,Rotation2d>(new Pose2d(x, y, Rotation2d.fromRadians(poseRadians)), Rotation2d.fromRadians(headingRadians)));
    }

    /**
     * Creates a {@link Pose2d} that is mirrored if the robot is on the red alliance.
     * @param x pose x
     * @param y pose y
     * @param rotation2d pose rotation
     * @return the pose that is mirrored if necessary
     */
    public static Pose2d poseFromRotation2d(double x, double y, Rotation2d rotation2d) {
        return mirror(new Pose2d(x, y, rotation2d));
    }

    /**
     * Creates a {@link Pose2d} that is mirrored if the robot is on the red alliance.
     * @param x pose x
     * @param y pose y
     * @param degrees pose rotation
     * @return the pose that is mirrored if necessary
     */
    public static Pose2d poseFromDegrees(double x, double y, double degrees) {
        return mirror(new Pose2d(x, y, Rotation2d.fromDegrees(degrees)));
    }

    /**
     * Creates a {@link Pose2d} that is mirrored if the robot is on the red alliance.
     * @param x pose x
     * @param y pose y
     * @param radians pose rotation
     * @return the pose that is mirrored if necessary
     */
    public static Pose2d poseFromRadians(double x, double y, double radians) {
        return mirror(new Pose2d(x, y, Rotation2d.fromRadians(radians)));
    }

    /**
     * Creates a {@link Rotation2d} that is mirrored if the robot is on the red alliance.
     * @param degrees rotation
     * @return the rotation that is mirrored if necessary
     */
    public static Rotation2d rotationFromDegrees(double degrees) {
        return mirror(Rotation2d.fromDegrees(degrees));
    }

    /**
     * Creates a {@link Rotation2d} that is mirrored if the robot is on the red alliance.
     * @param radians rotation
     * @return the rotation that is mirrored if necessary
     */
    public static Rotation2d rotationFromRadians(double radians) {
        return mirror(Rotation2d.fromRadians(radians));
    }
}

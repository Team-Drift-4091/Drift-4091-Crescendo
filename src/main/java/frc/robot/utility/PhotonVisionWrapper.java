// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Links: 
 * https://docs.photonvision.org/en/latest/docs/examples/index.html
 * https://docs.photonvision.org/en/latest/docs/examples/apriltag.html
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/PhotonCameraWrapper.java
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/Drivetrain.java
 */
public class PhotonVisionWrapper {
    private enum Pipeline{
        APRILTAG(0),
        CUBE(1),
        CONE(2);

        public final int index;

        private Pipeline(int pipeline) {
            this.index = pipeline;
        }

        public int getIndex(){
            return index;
        }
    }
    private Pipeline currentPipeline;

    private static PhotonVisionWrapper instance = null;
    public static PhotonVisionWrapper getInstance() {
        if (instance == null) {
            instance = new PhotonVisionWrapper();
        }
        return instance;
    }

    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    private boolean enabled = true;
    
    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    private PhotonVisionWrapper() {
        // Create a simple field layout.  This should be overwritten by reading from the json file, but it's necessary to have something just in case.
        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(List.of(
            new AprilTag(1, new Pose3d())
        ), Constants.FIELD_LENGTH, Constants.FIELD_WIDTH);

        // Try to read from the json file, and apply it to fieldLayout.
        try {
            fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toString()+"/2023-chargedup.json");
        } catch (IOException e) {
            DriverStation.reportError(e.toString(), false);
        }

        photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.CAMERA_TO_ROBOT);

        setPipeline(Pipeline.CUBE);
    }

    public Optional<PhotonTrackedTarget> getCubeLocation() {
        if (currentPipeline == Pipeline.CUBE) {
            PhotonTrackedTarget bestTarget = photonCamera.getLatestResult().getBestTarget();
            if (bestTarget == null) {
                return Optional.empty();
            }
            return Optional.of(bestTarget);
        }
        return Optional.empty();
    }

    public boolean seesCube() {
        return currentPipeline == Pipeline.CUBE && getCubeLocation().isPresent();
    }
    
    /**
     * Returns the result of the PhotonPoseEstimator after accounting for the last estimated pose of the drivetrain
     * @param prevEstimatedRobotPose the last estimated pose from the drivetrain
     * @return the new estimated pose using vision.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (enabled && currentPipeline == Pipeline.APRILTAG) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update();    
        }
        return Optional.empty();
    }

    public Pipeline getPipeline() {
        return currentPipeline;
    }

    /**
     * Sets the pipeline to the desired pipeline
     * @param pipeline the pipeline for the photonvision processing to use
     */
    public void setPipeline(Pipeline pipeline) {
        currentPipeline = pipeline;
        photonCamera.setPipelineIndex(pipeline.getIndex());
    }
}

package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
// import org.photonvision.estimation.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class VisionLocalization {
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;
    // define the camera's position relative to the robot
    private Transform3d robotToCam = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0, 0, 0)
    );
    
    public VisionLocalization(String cameraName) {
        // load the field json layout of apriltags
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
        // initialize the camera
        camera = new PhotonCamera(cameraName);
        
        //create the PhotonPoseEstimator with the field layout, strategy of
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, //json layout of april tags
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, //strategy to calculate pose position

            // ********instead of using a strategy get the raw value by dividing. somewhere in the docs.
            // *******set the value using that and use this strategy later on in the last part with parameters

            robotToCam //relative position of camera to robot
        );
        
    }

    // second VisionLocalization constructor that sets the
    public VisionLocalization(String cameraName, Translation3d translation, Rotation3d rotation) {
        // load the field json layout of apriltags
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
        // initialize the camera
        camera = new PhotonCamera(cameraName);
        
        //create the PhotonPoseEstimator with the field layout, strategy of
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, //json lahyout of april tags
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, //strategy to calculate pose position
            robotToCam //relative position of camera to robot
        );
        
        // Set robotTOCam
        this.robotToCam = new Transform3d(
            translation,
            rotation
        );
    }
    

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
        // Update the reference pose if using the CLOSEST_TO_REFERENCE_POSE strategy
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        // Get the latest result from the camera
        PhotonPipelineResult cameraResult = camera.getLatestResult();
        if (cameraResult == null || !cameraResult.hasTargets()) {
            return Optional.empty(); // No targets, return empty
        }
    
        // Get the estimated pose from the pose estimator
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update(cameraResult);
    
        // If result is present, process it
        if (result.isPresent()) {
            // Transform the estimated pose based on the robot-to-camera transformation
            Pose3d transformedPose = result.get().estimatedPose.transformBy(robotToCam);
    
            // Create a new EstimatedRobotPose with the transformed pose
            EstimatedRobotPose updatedPose = new EstimatedRobotPose(
                transformedPose,
                result.get().timestampSeconds, // Use the original timestamp
                result.get().targetsUsed,         // Pass through targets
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE // Retain strategy
            );
    
            // Return the updated pose wrapped in Optional
            return Optional.of(updatedPose);
        }
    
        // Return empty if no pose was estimated
        return Optional.empty();
    }
    }

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
            aprilTagFieldLayout, //json lahyout of april tags
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, //strategy to calculate pose position
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
        //updates stored reference pose when you use the closest_to_reference_pose strategy
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        
        // get the latest result
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update(camera.getLatestResult());
        
        // a Pose3d to transform based on the robotToCam transformation
        Pose3d tempPose3d = result.get().estimatedPose.transformBy(robotToCam);
        
        // convert transformed result to an estimated RobotPose
        // The parameters for Estimated Poses have negative time stamp because we're not using it

        // We also set a strategy, but we're not sure if it impacts anything.
        // If it does we need to figure another way of changing it
        EstimatedRobotPose tempEstimatedPose = new EstimatedRobotPose(tempPose3d, -1.0, new ArrayList<PhotonTrackedTarget>(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        result = Optional.ofNullable(tempEstimatedPose);
        
        // return results
        if (result.isPresent()) {
            
            return result;
        } else {
            return Optional.empty();
        }
    }
}

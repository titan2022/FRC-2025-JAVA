package frc.robot.utility;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
// import org.photonvision.estimation.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
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
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
        //updates stored reference pose when you use the closest_to_reference_pose strategy
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> result = photonPoseEstimator.update(camera.getLatestResult());
        if (result.isPresent()) {
            return result;
        } else {
            return Optional.empty();
        }
    }
}

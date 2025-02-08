// Based on https://github.com/Mechanical-Advantage/AdvantageKit/releases/download/v4.1.0/AdvantageKit_VisionTemplate.zip
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025 FRC#2022
//                         2021-2025 FRC#6328

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.networking.NetworkingCall;
import frc.robot.utility.networking.NetworkingServer;
import frc.robot.utility.networking.types.NetworkingPose;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOTitanProcessing implements VisionIO {
//   private final Supplier<Rotation2d> rotationSupplier;
//   private final DoubleArrayPublisher orientationPublisher;

//   private final DoubleSubscriber latencySubscriber;
//   private final DoubleSubscriber txSubscriber;
//   private final DoubleSubscriber tySubscriber;
//   private final DoubleArraySubscriber megatag1Subscriber;
//   private final DoubleArraySubscriber megatag2Subscriber;

  private final NetworkingServer server;
  
  private long updateTime; // based on RobotController.getFPGATime()
  private Pose3d robotPose;

  /**
   * Creates a new VisionIOTitanProcessing.
   *
   * @param server The NetworkingServer
   */
  public VisionIOTitanProcessing(int port) {
    server = new NetworkingServer(port);

    setup();
  }

  /**
   * Updates first frame localization estimates
   */
  private void setup() {
    // resetHeading();

    if (server != null) {
      server.subscribe("pose", (NetworkingCall<NetworkingPose>)(NetworkingPose pose) -> {
          SmartDashboard.putNumber("poseX", pose.position.getX());
          SmartDashboard.putNumber("poseZ", pose.position.getZ());
          // FIXME: Have the coprocessor send the timestamp of the update in UDP
          updateTime = RobotController.getFPGATime();
          robotPose = new Pose3d(pose.position, pose.rotation);
      });

      // server.subscribe("speaker",  (NetworkingCall<NetworkingPose>)(NetworkingPose speaker) -> {
      //     speakerHeading = Rotation2d.fromRadians(speaker.rotation.getY());
      //     SmartDashboard.putNumber("speakerHeading", speakerHeading.getDegrees());
      // });

      // server.subscribe("visible", (NetworkingCall<Translation3d>)(Translation3d fakeVec) -> {
      //     speakerTagVisible = fakeVec.getX() > 0;
      //     SmartDashboard.putBoolean("Tag Visible", speakerTagVisible);
      // });
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = ((RobotController.getFPGATime() - updateTime) / 1000) < 250;

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // var multitagResult = result.multitagResult.get();

    // // Calculate robot pose
    // Transform3d fieldToCamera = multitagResult.estimatedPose.best;
    // Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
    // Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

    // // Calculate average tag distance
    // double totalTagDistance = 0.0;
    // for (var target : result.targets) {
    //   totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    // }

    // // Add tag IDs
    // tagIds.addAll(multitagResult.fiducialIDsUsed);

    // Add observation
    poseObservations.add(
        new PoseObservation(
            updateTime, // Timestamp
            robotPose, // 3D pose estimate
            0, // Ambiguity = 0 (FIXME)
            1, // We don't know how many tags there are but it doesn't matter
            1, // Average tag distance = 1 (FIXME)
            PoseObservationType.TITANPROCESSING)); // Observation type

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // We don't send tags over UDP
    inputs.tagIds = new int[0];

    // // Save tag IDs to inputs objects
    // inputs.tagIds = new int[tagIds.size()];
    // int i = 0;
    // for (int id : tagIds) {
    //   inputs.tagIds[i++] = id;
    // }
  }
}

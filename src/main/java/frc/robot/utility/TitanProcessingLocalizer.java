package frc.robot.utility;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.networking.NetworkingCall;
import frc.robot.utility.networking.NetworkingServer;
import frc.robot.utility.networking.types.NetworkingPose;

public class TitanProcessingLocalizer extends Localizer {
  private final NetworkingServer server;
  private LocalizerMeasurement measurement = new LocalizerMeasurement(new Pose2d(), Double.NEGATIVE_INFINITY);

  public TitanProcessingLocalizer(int port) {
    server = new NetworkingServer(port);

    if (server != null) {
      server.subscribe("pose", (NetworkingCall<NetworkingPose>)(NetworkingPose pose) -> {
        SmartDashboard.putNumber("poseX", pose.position.getX());
        SmartDashboard.putNumber("poseZ", pose.position.getZ());
        // FIXME: Have the coprocessor send the timestamp of the update in UDP
        double timestamp = Utils.getCurrentTimeSeconds();
        Pose2d pose2d = new Pose2d(new Translation2d(pose.position.getX(), pose.position.getY()), new Rotation2d(pose.rotation.getZ()));
        measurement = new LocalizerMeasurement(pose2d, timestamp);

        publishMeasurement(measurement);
      });
    }
  }

  public NetworkingServer getServer() {
    return server;
  }

  /**
   * Get the localizer's type.
   */
  public LocalizerType getType() {
    return LocalizerType.TITANPROCESSING;
  }

  public LocalizerMeasurement getMeasurement() {
    return measurement;
  }

  /**
   * Updates the state of the localization estimates
   */
  public void step() {
    // no-op
  }
}

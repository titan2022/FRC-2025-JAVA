package frc.robot.utility;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utility.Localizer.LocalizerMeasurement;
import frc.robot.utility.networking.NetworkingCall;

public class Localizers {
  private final OdometryLocalizer odometryLocalizer;
  private final TitanProcessingLocalizer visionLocalizer;
  private NetworkingCall<LocalizerMeasurement> visionSubscriber;
  private boolean isMixed = false;
  private boolean hasSubscribedToVision = false;

  private final boolean HAS_TITAN_PROCESSING = true;

  public Localizers(OdometryLocalizer odometryLocalizer, TitanProcessingLocalizer visionLocalizer) {
    this.odometryLocalizer = odometryLocalizer;
    this.visionLocalizer = visionLocalizer;

    if(HAS_TITAN_PROCESSING) {
      visionSubscriber = (LocalizerMeasurement measurement) -> {
        if(isMixed) {
          odometryLocalizer.addVisionMeasurement(measurement);
        }
      };
    }
  }

  public OdometryLocalizer getOdometry() {
    return odometryLocalizer;
  }

  public TitanProcessingLocalizer getVision() {
    return visionLocalizer;
  }

  /**
   * Start mixing the vision measurements into the odometry measurements.
   */
  public void enableMixing() {
    isMixed = true;
    odometryLocalizer.setIsMixed(true);
    if(!hasSubscribedToVision) {
      visionLocalizer.subscribe(visionSubscriber);
      hasSubscribedToVision = true;
    }
  }

  public void disableMixing() {
    isMixed = false;
    odometryLocalizer.setIsMixed(false);
  }

  public void disableMixingByUnsubscribing() {
    disableMixing();
    visionLocalizer.removeSubscriber(visionSubscriber);
    hasSubscribedToVision = false;
  }


  BooleanPublisher getMeasurementPublisher = NetworkTableInstance.getDefault().getBooleanTopic("vision - can get measurement").publish();
  DoublePublisher latencyPublisher = NetworkTableInstance.getDefault().getDoubleTopic("vision - latency").publish();
  DoublePublisher timeSincePublisher = NetworkTableInstance.getDefault().getDoubleTopic("vision - time since last message").publish();

  /**
   * Updates the state of the localization estimates
   */
  public void step() {
    if(HAS_TITAN_PROCESSING) {
      if (visionLocalizer.getMeasurement() == null) {
        getMeasurementPublisher.set(false);
        // SmartDashboard.putBoolean("vision - can get measurement", false);
        return;
      }
      getMeasurementPublisher.set(true);
      latencyPublisher.set(visionLocalizer.getMeasurement().getLatency());
      timeSincePublisher.set(visionLocalizer.getMeasurement().getTimeSince());
      // SmartDashboard.putBoolean("vision - can get measurement", true);
      // SmartDashboard.putNumber("vision - latency", visionLocalizer.getMeasurement().getLatency());
      // SmartDashboard.putNumber("vision - time since last message", visionLocalizer.getMeasurement().getTimeSince());
      visionLocalizer.step();
    }
    odometryLocalizer.step();
  }
}

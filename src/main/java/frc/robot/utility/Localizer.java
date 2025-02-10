package frc.robot.utility;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.utility.networking.NetworkingCall;
import frc.robot.utility.networking.NetworkingObserver;

public abstract class Localizer {
  public enum LocalizerType {
    ODOMETRY,
    TITANPROCESSING,
    MIXED
  }

  /**
   * Get the localizer's type.
   */
  abstract public LocalizerType getType();

  public class LocalizerMeasurement {
    /**
     * The pose of the robot in the WPILib coordinate system.
     */
    public final Pose2d pose;
    /**
     * The time of measurement, with the same epoch as Utils.getCurrentTimeSeconds.
     */
    public final double measurementTime;

    /**
     * The time the message arrived at the server, with the same epoch as Utils.getCurrentTimeSeconds.
     */
    public final double serverTime;

    public LocalizerMeasurement(Pose2d pose, double measurementTime, double serverTime) {
      this.pose = pose;
      this.measurementTime = measurementTime;
      this.serverTime = serverTime;
    }

    public LocalizerMeasurement(Pose2d pose, double measurementTime) {
      this.pose = pose;
      this.measurementTime = measurementTime;
      this.serverTime = Utils.getCurrentTimeSeconds();
    }

    public double getLatency() {
      return serverTime - measurementTime;
    }

    public double getTimeSince() {
      return Utils.getCurrentTimeSeconds() - serverTime;
    }
  }

  /**
   * Get the latest localizer measurement.
   */
  abstract public LocalizerMeasurement getMeasurement();

  /**
   * Updates the state of the localization estimates
   */
  abstract public void step();

  // Non-abstract parts begin here.

  private ArrayList<NetworkingCall<LocalizerMeasurement>> subscribers;

  /**
   * Subscribe to receive localizer measurements.
   */
  public void subscribe(NetworkingCall<LocalizerMeasurement> subscriber) {
    subscribers.add(subscriber);
  }

  /**
   * Remove a subscriber (Warning: this is an O(n) operation.)
   */
  public void removeSubscriber(NetworkingCall<LocalizerMeasurement> subscriber) {
    subscribers.remove(subscriber);
  }

  /**
   * Publish a measurement to the subscribers.
   */
  protected void publishMeasurement(LocalizerMeasurement measurement) {
    for(NetworkingCall<LocalizerMeasurement> subscriber : subscribers) {
      subscriber.update(measurement);
    }
  }

  /**
   * Publish the current measurement to the subscribers.
   */
  protected void publishCurrentMeasurement() {
    publishMeasurement(getMeasurement());
  }
}

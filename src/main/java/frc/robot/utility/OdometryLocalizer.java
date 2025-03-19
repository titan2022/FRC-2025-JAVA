package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class OdometryLocalizer extends Localizer {
  private final CommandSwerveDrivetrain drivetrain;
  private boolean isMixed;

  public OdometryLocalizer(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /**
   * Get the localizer's type.
   */
  public LocalizerType getType() {
    if(isMixed) {
      return LocalizerType.MIXED;
    } else {
      return LocalizerType.ODOMETRY;
    }
  }

  public boolean isMixed() {
    return isMixed;
  }

  public void setIsMixed(boolean isMixed) {
    this.isMixed = isMixed;
  }

  public void addVisionMeasurement(LocalizerMeasurement measurement) {
    drivetrain.addVisionMeasurement(measurement.pose, measurement.measurementTime);
  }

  public LocalizerMeasurement getMeasurement() {
    return new LocalizerMeasurement(drivetrain.getState().Pose, 0, drivetrain.getState().Timestamp);
  }

  /**
   * Updates the state of the localization estimates
   */
  public void step() {
    publishCurrentMeasurement();
  }
}

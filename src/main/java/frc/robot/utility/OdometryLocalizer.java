package frc.robot.utility;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class OdometryLocalizer extends Localizer {
  private final CommandSwerveDrivetrain drivetrain;
  private boolean isMixed;
  
  StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault().getStructTopic("localizerVisionPose", Pose2d.struct).publish();

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
    if (measurement.distance > 2.0) {
      return;
    }
    Matrix<N3, N1> stdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
    stdDevs.set(0, 0, 0.1 * measurement.distance);
    stdDevs.set(1, 0, 0.1 * measurement.distance);
    stdDevs.set(2, 0, 0.1 * measurement.distance );
    drivetrain.addVisionMeasurement(measurement.pose, measurement.measurementTime, stdDevs);
  }

  public LocalizerMeasurement getMeasurement() {
    posePub.set(drivetrain.getState().Pose);
    return new LocalizerMeasurement(drivetrain.getState().Pose, 0, drivetrain.getState().Timestamp);
  }

  /**
   * Updates the state of the localization estimates
   */
  public void step() {
    publishCurrentMeasurement();
  }
}

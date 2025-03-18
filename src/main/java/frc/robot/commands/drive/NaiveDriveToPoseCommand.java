package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.utility.Localizer;
import frc.robot.utility.ReefLocations;
import frc.robot.utility.Constants.Unit;

public class NaiveDriveToPoseCommand extends Command {
  private final Pose2d target;
  private final CommandSwerveDrivetrain drivetrain;
  private final Localizer localizer;

  /// The max speed, in meters per second
  public static final double MAX_SPEED_AUTOALIGN = 6.0; // m/s
  public static final double MAX_ACCELERATION_AUTOALIGN = 2.0; // m/s^2
  /// The max angular speed, in radians per second
  public static final double MAX_ANGULAR_SPEED_AUTOALIGN = 200.0 * Unit.DEG; // rad/s
  public static final double MAX_ANGULAR_ACCELERATION_AUTOALIGN = 200.0 * Unit.DEG; // rad/s^2

  public static final double FINISH_DEADBAND = 0.25; // m
  public static final double FINISH_ANGULAR_DEADBAND = 25 * Unit.DEG; // rad

  private final ProfiledPIDController pidX = new ProfiledPIDController(
    0.4, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidY = new ProfiledPIDController(
    0.4, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidTheta = new ProfiledPIDController(
    0.1, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED_AUTOALIGN,
      MAX_ANGULAR_ACCELERATION_AUTOALIGN
    )
  );

  public NaiveDriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Localizer localizer, Pose2d target) {
    this.target = target;
    this.drivetrain = drivetrain;
    this.localizer = localizer;
  }

  public static NaiveDriveToPoseCommand driveToNearestLeftScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    return new NaiveDriveToPoseCommand(drivetrain, localizer,
      ReefLocations.nearestLeftScoringLocation(localizer.getMeasurement().pose)
    );
  }

  public static NaiveDriveToPoseCommand driveToNearestRightScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    return new NaiveDriveToPoseCommand(drivetrain, localizer,
      ReefLocations.nearestRightScoringLocation(localizer.getMeasurement().pose)
    );
  }

  private Pose2d getMeasurement() {
    return localizer.getMeasurement().pose;
  }

  @Override
  public void initialize() {}
  
  StructPublisher<ChassisSpeeds> chassisPub = NetworkTableInstance.getDefault().getStructTopic("autoAlignVel", ChassisSpeeds.struct).publish();

  @Override
  public void execute() {
    Pose2d measurement = getMeasurement();
    ChassisSpeeds test = new ChassisSpeeds(
      pidX.calculate(measurement.getX(), target.getX()),
      pidY.calculate(measurement.getY(), target.getY()),
      pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
    );
    chassisPub.set(test);
    drivetrain.setVelocities(test);
  }

  StructPublisher<Pose2d> publisherTarget = NetworkTableInstance.getDefault().getStructTopic("targetAuto", Pose2d.struct).publish();
  StructPublisher<Transform2d> publisherDiff = NetworkTableInstance.getDefault().getStructTopic("autoDiff", Transform2d.struct).publish();

  @Override
  public boolean isFinished() {
    Pose2d measurement = getMeasurement();
    publisherTarget.set(target);
    publisherDiff.set(measurement.minus(target));
    return (measurement.minus(target).getTranslation().getNorm() <= FINISH_DEADBAND) && (Math.abs(measurement.minus(target).getRotation().getRadians()) <= FINISH_ANGULAR_DEADBAND);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setVelocities(new ChassisSpeeds(0, 0, 0));
  }
}

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private boolean isLeftSide = false;
  private final CommandSwerveDrivetrain drivetrain;
  private final Localizer localizer;

  /// The max speed, in meters per second
  public static final double MAX_SPEED_AUTOALIGN = 2.0; // m/s
  public static final double MAX_ACCELERATION_AUTOALIGN = 2.0; // m/s^2
  /// The max angular speed, in radians per second
  public static final double MAX_ANGULAR_SPEED_AUTOALIGN = 200.0 * Unit.DEG; // rad/s
  public static final double MAX_ANGULAR_ACCELERATION_AUTOALIGN = 200.0 * Unit.DEG; // rad/s^2

  public static final double FINISH_DEADBAND = 0.05; // m
  public static final double FINISH_ANGULAR_DEADBAND = 10 * Unit.DEG; // rad

  private final ProfiledPIDController pidX = new ProfiledPIDController(
    10.0, // kP
    0.0, // kI
    1.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidY = new ProfiledPIDController(
    10.0, // kP
    0.0, // kI
    1.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidTheta = new ProfiledPIDController(
    5, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED_AUTOALIGN,
      MAX_ANGULAR_ACCELERATION_AUTOALIGN
    )
  );

  public NaiveDriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
  }

  public static NaiveDriveToPoseCommand driveToNearestLeftScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    // isLeftSide = true;
    return new NaiveDriveToPoseCommand(drivetrain, localizer);
    // return new NaiveDriveToPoseCommand(drivetrain, localizer,
    //   new Pose2d(
    //     new Translation2d(1, 0.5),
    //     new Rotation2d(0)
    //   )
    // );
  }

  public NaiveDriveToPoseCommand driveToNearestRightScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    isLeftSide = false;
    return new NaiveDriveToPoseCommand(drivetrain, localizer);
  }

  private Pose2d getMeasurement() {
    return localizer.getMeasurement().pose;
    // return new Pose2d(
    //   new Translation2d(0, 0),
    //   new Rotation2d(0)
    // );
  }

  private Pose2d getTarget() {
    // if (isLeftSide) {
    //   return ReefLocations.nearestScoringLocation(localizer.getMeasurement().pose);
    // }
    // return ReefLocations.nearestRightScoringLocation(localizer.getMeasurement().pose);
    return ReefLocations.nearestScoringLocation(localizer.getMeasurement().pose);
  }

  @Override
  public void initialize() {}
  
  StructPublisher<ChassisSpeeds> chassisPub = NetworkTableInstance.getDefault().getStructTopic("autoAlignVel", ChassisSpeeds.struct).publish();
  StructPublisher<ChassisSpeeds> chassisPub2 = NetworkTableInstance.getDefault().getStructTopic("nonPIDautoAlignVel", ChassisSpeeds.struct).publish();

  @Override
  public void execute() {
    Pose2d measurement = getMeasurement();
    Pose2d target = getTarget();
    ChassisSpeeds test = new ChassisSpeeds(
      pidX.calculate(measurement.getX(), target.getX()),
      pidY.calculate(measurement.getY(), target.getY()),
      pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
    );
    // ChassisSpeeds test2 = new ChassisSpeeds(
    //   1.0 * (target.getY() - measurement.getY()),
    //   1.0 * (target.getX() - measurement.getX()),
    //   pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
    // );
    // chassisPub.set(test);
    // chassisPub2.set(test2);
    drivetrain.setFieldVelocities(test);








    // Translation2d vel = new Translation2d(
    //   pidX.calculate(measurement.getX(), target.getX()),
    //   pidY.calculate(measurement.getY(), target.getY())
    // ).rotateBy(measurement.getRotation());

    // ChassisSpeeds speeds = new ChassisSpeeds(
    //   vel.getX(),
    //   vel.getY(),
    //   pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
    // );

    // chassisPub.set(speeds);
    // drivetrain.setVelocities(speeds);
  }

  StructPublisher<Pose2d> publisherTarget = NetworkTableInstance.getDefault().getStructTopic("targetAuto", Pose2d.struct).publish();
  StructPublisher<Transform2d> publisherDiff = NetworkTableInstance.getDefault().getStructTopic("autoDiff", Transform2d.struct).publish();

  @Override
  public boolean isFinished() {
    Pose2d measurement = getMeasurement();
    Pose2d target = getTarget();
    publisherTarget.set(target);
    publisherDiff.set(target.minus(measurement));
    return (measurement.minus(target).getTranslation().getNorm() <= FINISH_DEADBAND) && (Math.abs(measurement.minus(target).getRotation().getRadians()) <= FINISH_ANGULAR_DEADBAND);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setVelocities(new ChassisSpeeds(0, 0, 0));
  }
}

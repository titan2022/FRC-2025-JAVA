package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.utility.Localizer;
import frc.robot.utility.ReefLocations;
import frc.robot.utility.Constants.Unit;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecond;


public class NaiveDriveToPoseCommand extends Command {
  private boolean isLeftSide = false;
  private final CommandSwerveDrivetrain drivetrain;
  private final Localizer localizer;

  /// The max speed, in meters per second
  public static final double MAX_SPEED_AUTOALIGN = 4.0; // m/s
  public static final double MAX_ACCELERATION_AUTOALIGN = 4.0; // m/s^2
  /// The max angular speed, in radians per second
  public static final double MAX_ANGULAR_SPEED_AUTOALIGN = 200.0 * Unit.DEG; // rad/s
  public static final double MAX_ANGULAR_ACCELERATION_AUTOALIGN = 200.0 * Unit.DEG; // rad/s^2

  public static final double FINISH_DEADBAND = 0.025; // m
  public static final double FINISH_ANGULAR_DEADBAND = 2 * Unit.DEG; // rad
  
  public Pose2d target;

  private final ProfiledPIDController pidX = new ProfiledPIDController(
    2.5, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidY = new ProfiledPIDController(
    2.5, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidTheta = new ProfiledPIDController(
    2, // kP
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

  public static NaiveDriveToPoseCommand driveToNearestScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
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
  public void initialize() {
    pidTheta.enableContinuousInput(0, 2 * Math.PI);
    Pose2d measurement = getMeasurement();
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, measurement.getRotation());
    target = getTarget();
    pidX.setGoal(target.getX());
    pidY.setGoal(target.getY());
    pidTheta.setGoal(target.getRotation().getRadians());
    pidX.reset(new State(measurement.getX(), fieldSpeeds.vxMetersPerSecond));
    pidY.reset(new State(measurement.getY(), fieldSpeeds.vyMetersPerSecond));
    pidTheta.reset(new State(measurement.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond));
  }
  
  StructPublisher<ChassisSpeeds> chassisPub = NetworkTableInstance.getDefault().getStructTopic("autoAlignVel", ChassisSpeeds.struct).publish();

  @Override
  public void execute() {
    Pose2d measurement = getMeasurement();
    ChassisSpeeds test = new ChassisSpeeds(
      pidX.calculate(measurement.getX(), target.getX()),
      pidY.calculate(measurement.getY(), target.getY()),
      pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
    );

    SmartDashboard.putNumberArray("X PID", new Double[] {measurement.getX(), target.getX(), pidX.calculate(measurement.getX(), target.getX())});
    chassisPub.set(test);
    drivetrain.setFieldVelocities(test);

    // Pose2d robotPose = drivetrain.getState().Pose;

    // drivetrain.setControl(new SwerveRequest.FieldCentric()
    //   .withDriveRequestType(DriveRequestType.Velocity)
    //   .withVelocityX(-pidX.calculate(robotPose.getX()))
    //   .withVelocityY(-pidY.calculate(robotPose.getY()))
    //   .withDeadband(InchesPerSecond.of(5))
    //   //.withRotationalRate(pidTheta.calculate(robotPose.getRotation().getRadians()))
    //   //.withRotationalDeadband(DegreesPerSecond.of(15))
    //   //.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    // );
    SmartDashboard.putNumber("NaiveDriveToPoseCommand target x-velocity", pidX.getSetpoint().velocity);
    SmartDashboard.putNumber("NaiveDriveToPoseCommand target y-velocity", pidY.getSetpoint().velocity);
    SmartDashboard.putNumber("NaiveDriveToPoseCommand target theta-velocity", pidTheta.getSetpoint().velocity);
  }

  StructPublisher<Pose2d> publisherTarget = NetworkTableInstance.getDefault().getStructTopic("targetAuto", Pose2d.struct).publish();
  StructPublisher<Transform2d> publisherDiff = NetworkTableInstance.getDefault().getStructTopic("autoDiff", Transform2d.struct).publish();

  @Override
  public boolean isFinished() {
    // Pose2d measurement = getMeasurement();
    // Pose2d target = getTarget();
    publisherTarget.set(target);
    // publisherDiff.set(target.minus(measurement));
    // return (measurement.minus(target).getTranslation().getNorm() <= FINISH_DEADBAND) && (Math.abs(measurement.minus(target).getRotation().getRadians()) <= FINISH_ANGULAR_DEADBAND);
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
  }
}

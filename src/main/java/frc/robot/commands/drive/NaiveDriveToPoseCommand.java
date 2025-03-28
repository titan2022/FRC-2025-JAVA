package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Localizer;
import frc.robot.utility.ReefLocations;


public class NaiveDriveToPoseCommand extends Command {
  private boolean isLeftSide = false;
  private boolean isL1 = false;
  private final CommandSwerveDrivetrain drivetrain;
  private final Localizer localizer;

  /// The max speed, in meters per second
  public static final double MAX_SPEED_AUTOALIGN = 4; // m/s
  public static final double MAX_ACCELERATION_AUTOALIGN = 4; // m/s^2
  /// The max angular speed, in radians per second
  public static final double MAX_ANGULAR_SPEED_AUTOALIGN = 200.0 * Unit.DEG; // rad/s
  public static final double MAX_ANGULAR_ACCELERATION_AUTOALIGN = 200.0 * Unit.DEG; // rad/s^2

  public static final double FINISH_DEADBAND = 1 * Unit.IN;
  public static final double FINISH_ANGULAR_DEADBAND = 2 * Unit.DEG;
  
  public Pose2d target;

  private final ProfiledPIDController pidX = new ProfiledPIDController(
    4, // kP
    0, // kI
    0.5, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidY = new ProfiledPIDController(
    4, // kP
    0, // kI
    0.5, // kD
    new TrapezoidProfile.Constraints( 
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidTheta = new ProfiledPIDController(
    2, // kP
    0.0, // kI
    0.25, // kD
    new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED_AUTOALIGN,
      MAX_ANGULAR_ACCELERATION_AUTOALIGN
    )
  );

  public NaiveDriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Localizer localizer, boolean isLeftSide) {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.isLeftSide = isLeftSide;
  }

  public NaiveDriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Localizer localizer, boolean isLeftSide, boolean isL1) {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.isLeftSide = isLeftSide;
    this.isL1 = isL1;
  }

  public static NaiveDriveToPoseCommand driveToNearestLeftScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    return new NaiveDriveToPoseCommand(drivetrain, localizer, true);
  }

  public static NaiveDriveToPoseCommand driveToNearestRightScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    return new NaiveDriveToPoseCommand(drivetrain, localizer, false);
  }

  public static NaiveDriveToPoseCommand driveToNearestLeftL1ScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    return new NaiveDriveToPoseCommand(drivetrain, localizer, true, true);
  }

  public static NaiveDriveToPoseCommand driveToNearestRightL1ScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    return new NaiveDriveToPoseCommand(drivetrain, localizer, false, true);
  }

  private Pose2d getMeasurement() {
    return localizer.getMeasurement().pose;
    // return new Pose2d(
    //   new Translation2d(0, 0),
    //   new Rotation2d(0)
    // );
  }

  private Pose2d getTarget() {
    if(isL1){
      if (isLeftSide) {
        return ReefLocations.nearestLeftL1ScoringLocation(localizer.getMeasurement().pose);
      }
      return ReefLocations.nearestRightL1ScoringLocation(localizer.getMeasurement().pose);
    }
    if (isLeftSide) {
      return ReefLocations.nearestLeftScoringLocation(localizer.getMeasurement().pose);
    }
    return ReefLocations.nearestRightScoringLocation(localizer.getMeasurement().pose);
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
    //publisherTarget.set(target);
  }
  
  // StructPublisher<ChassisSpeeds> chassisPub = NetworkTableInstance.getDefault().getStructTopic("autoAlignVel", ChassisSpeeds.struct).publish();

  @Override
  public void execute() {
    Pose2d measurement = getMeasurement();
    ChassisSpeeds test = new ChassisSpeeds(
      pidX.calculate(measurement.getX(), target.getX()),
      pidY.calculate(measurement.getY(), target.getY()),
      pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
    );

    // SmartDashboard.putNumberArray("X PID", new Double[] {measurement.getX(), target.getX(), pidX.calculate(measurement.getX(), target.getX())});
    // chassisPub.set(test);
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
    // SmartDashboard.putNumber("NaiveDriveToPoseCommand target x-velocity", pidX.getSetpoint().velocity);
    // SmartDashboard.putNumber("NaiveDriveToPoseCommand target y-velocity", pidY.getSetpoint().velocity);
    // SmartDashboard.putNumber("NaiveDriveToPoseCommand target theta-velocity", pidTheta.getSetpoint().velocity);
  }

  StructPublisher<Pose2d> publisherTarget = NetworkTableInstance.getDefault().getStructTopic("targetAuto", Pose2d.struct).publish();
  // StructPublisher<Transform2d> publisherDiff = NetworkTableInstance.getDefault().getStructTopic("autoDiff", Transform2d.struct).publish();

  @Override
  public boolean isFinished() {
    // Pose2d measurement = getMeasurement();
    // Pose2d target = getTarget();
    publisherTarget.set(target);
    // publisherDiff.set(target.minus(measurement));
    //return (measurement.minus(target).getTranslation().getNorm() <= FINISH_DEADBAND) && (Math.abs(measurement.minus(target).getRotation().getRadians()) <= FINISH_ANGULAR_DEADBAND);
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
  }
}

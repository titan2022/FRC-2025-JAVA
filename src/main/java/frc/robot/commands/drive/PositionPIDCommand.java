package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.security.KeyStore.LoadStoreParameter;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Localizer;

public class PositionPIDCommand extends Command{
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Localizer localizer;

    public final Pose2d target;

    public static final double MAX_SPEED_AUTOALIGN = 4; // m/s
    public static final double MAX_ACCELERATION_AUTOALIGN = 4; // m/s^2
    /// The max angular speed, in radians per second
    public static final double MAX_ANGULAR_SPEED_AUTOALIGN = 200.0 * Unit.DEG; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION_AUTOALIGN = 200.0 * Unit.DEG; // rad/s^2

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final double endTriggerDebounce = 0.04;

    public static final double FINISH_DEADBAND = 1 * Unit.IN;
    public static final double FINISH_ANGULAR_DEADBAND = 2 * Unit.DEG;
    public static final LinearVelocity FINISH_SPEED_DEADBAND = InchesPerSecond.of(2);

     private final ProfiledPIDController pidX = new ProfiledPIDController(
    4, // kP
    0.0, // kI
    0.5, // kD
    new TrapezoidProfile.Constraints(
      MAX_SPEED_AUTOALIGN,
      MAX_ACCELERATION_AUTOALIGN
    )
  );

  private final ProfiledPIDController pidY = new ProfiledPIDController(
    4, // kP
    0.0, // kI
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



    private PositionPIDCommand(CommandSwerveDrivetrain drivetrain, Localizer localizer, Pose2d target) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.target = target;
        endTrigger = new Trigger(() -> {
            Pose2d diff = drivetrain.getState().Pose.relativeTo(target);

            var rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                FINISH_ANGULAR_DEADBAND, 
                0.0, 
                1.0
            );
    
            var position = diff.getTranslation().getNorm() < FINISH_DEADBAND;
    
            var speed = drivetrain.getState().Speeds.vxMetersPerSecond < FINISH_SPEED_DEADBAND.in(MetersPerSecond);
            
            return rotation && position && speed;
        });
        endTriggerDebounced = endTrigger.debounce(endTriggerDebounce);
    }

    public static Command generateCommand(CommandSwerveDrivetrain drivetrain, Localizer localizer, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(drivetrain, localizer, goalPose).withTimeout(timeout).finallyDo(() -> {
            drivetrain.brake();
        });
    }

    private Pose2d getMeasurement() {
        return localizer.getMeasurement().pose;
        // return new Pose2d(
        //   new Translation2d(0, 0),
        //   new Rotation2d(0)
        // );
    }

    @Override
    public void initialize() {
        pidTheta.enableContinuousInput(0, 2 * Math.PI);
        Pose2d measurement = getMeasurement();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, measurement.getRotation());
        pidX.setGoal(target.getX());
        pidY.setGoal(target.getY());
        pidTheta.setGoal(target.getRotation().getRadians());
        pidX.reset(new State(measurement.getX(), fieldSpeeds.vxMetersPerSecond));
        pidY.reset(new State(measurement.getY(), fieldSpeeds.vyMetersPerSecond));
        pidTheta.reset(new State(measurement.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond));
    }

    @Override
    public void execute() {
        Pose2d measurement = getMeasurement();
        ChassisSpeeds test = new ChassisSpeeds(
          pidX.calculate(measurement.getX(), target.getX()),
          pidY.calculate(measurement.getY(), target.getY()),
          pidTheta.calculate(measurement.getRotation().getRadians(), target.getRotation().getRadians())
        );
        drivetrain.setFieldVelocities(test);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
       
    }
}
package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Localizer;
import frc.robot.utility.ReefLocations;


public class PathPlannerAutoAlign{
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

  public static final Time AutoAlignAdjustTimeout = Seconds.of(0.6);

  public static final PathConstraints pathConstraints = new PathConstraints(2.0, 2.0, 1/2 * Math.PI, 1 * Math.PI); 

  

  public PathPlannerAutoAlign(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    this.drivetrain = drivetrain;
    this.localizer = localizer;
  }


  public Command driveToNearestLeftScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    Pose2d target = getTarget(true, false);
    return getPathFromWaypoint(getWaypointFromTarget(target));
  }

  public Command driveToNearestRightScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    Pose2d target = getTarget(false, false);
    return getPathFromWaypoint(getWaypointFromTarget(target));
  }

  public Command driveToNearestLeftL1ScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    Pose2d target = getTarget(true, true);
    return  getPathFromWaypoint(getWaypointFromTarget(target));
  }

  public Command driveToNearestRightL1ScoringLocation(CommandSwerveDrivetrain drivetrain, Localizer localizer) {
    Pose2d target = getTarget(false, true);
    return  getPathFromWaypoint(getWaypointFromTarget(target));
  }

  private Pose2d getMeasurement() {
    return localizer.getMeasurement().pose;

  }

  private Pose2d getTarget(boolean isLeftSide, boolean isL1) {
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

  private Pose2d getWaypointFromTarget(Pose2d target){
    return new Pose2d(
      target.getTranslation(),
      target.getRotation().rotateBy(Rotation2d.k180deg)
    );
}


  private Command getPathFromWaypoint(Pose2d waypoint) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(drivetrain.getState().Pose.getTranslation(), getPathVelocityHeading(drivetrain.getVelocities(), waypoint)),
        waypoint
    );

    if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
        return PositionPIDCommand.generateCommand(drivetrain, waypoint, AutoAlignAdjustTimeout);
        
    }

    PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        pathConstraints,
        new IdealStartingState(getVelocityMagnitude(drivetrain.getVelocities()), drivetrain.getState().Pose.getRotation()), 
        new GoalEndState(0.0, waypoint.getRotation())
    );

    path.preventFlipping = true;

    return AutoBuilder.followPath(path).andThen(
          PositionPIDCommand.generateCommand(drivetrain, waypoint, AutoAlignAdjustTimeout)
        );
  }

  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(drivetrain.getState().Pose).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }
  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

}

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

public class PositionPIDCommand extends Command{
    
    private final CommandSwerveDrivetrain drivetrain;
    public final Pose2d goalPose;
    private PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0),
        new PIDConstants(5, 0, 0)
    );

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(0.04);

    public static final double FINISH_DEADBAND = 1 * Unit.IN;
    public static final double FINISH_ANGULAR_DEADBAND = 2 * Unit.DEG;
    public static final LinearVelocity FINISH_SPEED_DEADBAND = InchesPerSecond.of(2);


    private PositionPIDCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;
    }

    public static Command generateCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(drivetrain, goalPose).withTimeout(timeout).finallyDo(() -> {
            drivetrain.setFieldVelocities(new ChassisSpeeds(0,0,0));
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        drivetrain.setFieldVelocities(
            driveController.calculateRobotRelativeSpeeds(
                drivetrain.getState().Pose, goalState
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {

        Pose2d diff = drivetrain.getState().Pose.relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            FINISH_ANGULAR_DEADBAND, 
            0.0, 
            1.0
        );

        var position = diff.getTranslation().getNorm() < FINISH_DEADBAND;

        var speed = drivetrain.getState().Speeds.vxMetersPerSecond < FINISH_SPEED_DEADBAND.in(MetersPerSecond);
        
        return endTriggerDebouncer.calculate(
            rotation && position && speed
        );
    }
}
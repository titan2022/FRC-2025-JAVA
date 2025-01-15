package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.networktables.Topic;

public class PoseLogger2d extends Command {
    private Pose2d currentPose;
    private Translation2d direction;
    private double speed;
    private double period;
    private final StructPublisher<Pose2d> currentPosePublisher;

    public PoseLogger2d(Pose2d initialPose, Translation2d direction, double speed, double period) {
        this.currentPose = initialPose;
        this.speed = speed;
        this.period = period;

        double magnitude = direction.getNorm();
        this.direction = (magnitude != 0) ? direction.div(magnitude) : new Translation2d(0, 0);
        
        Topic poseTopic = NetworkTableInstance.getDefault()
            .getTopic("PoseSimulation/CurrentPose");

        poseTopic.setRetained(true);

        this.currentPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("PoseSimulation/CurrentPose", Pose2d.struct)
            .publish();
    }

    @Override
    public void initialize() {
        System.out.println("PoseLogger2d initialized");
    }

    @Override
    public void execute() {
        Translation2d displacement = direction.times(speed * period);

        currentPose = new Pose2d(
            currentPose.getTranslation().plus(displacement),
            currentPose.getRotation()
        );

        System.out.println("Updated Pose: " + currentPose);
        System.out.println("Pose - X: " + currentPose.getTranslation().getX());
        System.out.println("Pose - Y: " + currentPose.getTranslation().getY());
        System.out.println("Pose - Rotation: " + currentPose.getRotation().getDegrees());


        currentPosePublisher.set(currentPose);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PoseLogger2d ended");
    }
}


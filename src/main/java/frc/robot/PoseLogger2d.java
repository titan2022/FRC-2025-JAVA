package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class PoseLogger2d extends Command {
    private Pose2d currentPose;
    private final Translation2d direction;
    private final double speed;
    private final double period;

    public PoseLogger2d(Pose2d initialPose, Translation2d direction, double speed, double period) {
        this.currentPose = initialPose;
        this.speed = speed;
        this.period = period;
        
        // Normalizes direction vector, and sets direction vector to 0 if magnitude is 0 to prevent division error
        double magnitude = direction.getNorm();
        this.direction = (magnitude != 0) ? direction.div(magnitude) : new Translation2d(0, 0);
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

        // Log the current pose
        Logger.recordOutput("PoseSimulation/CurrentPose", currentPose);
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

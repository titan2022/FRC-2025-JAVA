package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class SimulatePose3d extends Command {
    private Pose3d currentPose;
    private Translation3d direction;
    private double speed; 
    private double period; 

    public SimulatePose3d(Pose3d initialPose, Translation3d direction, double speed, double period) {
        this.currentPose = initialPose;
        this.speed = speed;
        this.period = period;

        double magnitude = Math.sqrt(direction.getX() * direction.getX() + direction.getY() * direction.getY() + direction.getZ() * direction.getZ());
        if (magnitude != 0) {
            this.direction = new Translation3d(direction.getX() / magnitude, direction.getY() / magnitude, direction.getZ() / magnitude);
        } else {
            this.direction = new Translation3d(0, 0, 0);
        }

        initialize();
        for (int i = 0; i < 100; i++) {
            execute();
        }        
    }
    
    @Override
    public void initialize() {
        Logger.recordOutput("PoseSimulation/InitialPose", currentPose);
    }

    @Override
    public void execute() {
        Translation3d displacement = direction.times(speed * period);

        currentPose = new Pose3d(
                currentPose.getTranslation().plus(displacement),
                currentPose.getRotation()
        );

        Logger.recordOutput("PoseSimulation/CurrentPose", currentPose);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Logger.recordOutput("PoseSimulation/Status", "Interrupted");
        } else {
            Logger.recordOutput("PoseSimulation/Status", "Completed");
        }
    }

    SimulatePose3d tester = new SimulatePose3d(
        new Pose3d(new Translation3d(0.5, 0.5, 0.5), new Rotation3d(0.5, 0.5, 0.5)),
        new Translation3d(1.5, 1.5, 1.5), 
        2, 2
    );
}

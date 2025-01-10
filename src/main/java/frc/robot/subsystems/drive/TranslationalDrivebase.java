package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A drivebase capable of omnidirectional translational motion.
 */
public interface TranslationalDrivebase extends Subsystem {
    /**
     * Sets the robot-oriented velocity in meters per second.
     * 
     * @param velocity The desired velocity in meters per second.
     */
    public void setVelocity(Translation2d velocity);

    /**
     * Returns the current robot-oriented velocity in meters per second.
     * 
     * @return The current velocity in meters per second.
     */
    public Translation2d getVelocity();

    public Command translationalDrive(CommandXboxController xbox);
}

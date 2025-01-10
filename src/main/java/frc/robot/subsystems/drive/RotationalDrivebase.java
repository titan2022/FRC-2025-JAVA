package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utility.Localizer;

/**
 * A drivebase capable of rotation.
 */
public interface RotationalDrivebase extends Subsystem {
    /**
     * Sets the robot-oriented rotational velocity in radians per second.
     * 
     * @param omega The desired rotational velocity in radians per second.
     */
    public void setRotationalVelocity(Rotation2d omega);

    /**
     * Returns the current robot-oriented rotational velocity in radians per second.
     * 
     * @return The current rotational velocity in radians per second.
     */
    public Rotation2d getRotationalVelocity();

    public Command rotationalDrive(CommandXboxController xbox);

}

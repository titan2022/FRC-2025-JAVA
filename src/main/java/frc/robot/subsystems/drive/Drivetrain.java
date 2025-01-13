package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drivetrain {
    public TranslationalDrivebase getTranslational();
    public RotationalDrivebase getRotational();

    public void setVelocities(ChassisSpeeds speeds);

    public ChassisSpeeds getVelocities();

    public void brake();
}

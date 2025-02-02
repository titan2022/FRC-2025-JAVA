package frc.robot.subsystems.drive;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class YAGSLSwerveDrivetrain extends SubsystemBase {
    private static File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private SwerveDrive swerveDrive;

    public YAGSLSwerveDrivetrain() {
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(5.0);
        } catch (Exception e) {
            System.err.println(
                    "Could not create swerve drive. Make sure src/main/deploy/swerve/controllerproperties.json, src/main/deploy/swerve/modules, and src/main/deploy/swerve/swervedrive.json exist.");
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.setModuleStateOptimization(false);
        // swerveDrive.pushOffsetsToEncoders();
    }

    public void setVelocities(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    public ChassisSpeeds getVelocities() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    false,
                    false);
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void brake() {
        setVelocities(new ChassisSpeeds(0, 0, 0));
        swerveDrive.setMotorIdleMode(true);
    }
}

package frc.robot.subsystems.drive;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utility.Constants;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class YAGSLSwerveDrivetrain extends SubsystemBase {
    private static File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private SwerveDrive swerveDrive;

    public YAGSLSwerveDrivetrain() {
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(TunerConstants.MAX_SPEED);
        } catch (Exception e) {
            System.err.println(
                    "Could not create swerve drive. Make sure src/main/deploy/swerve/controllerproperties.json, src/main/deploy/swerve/modules, and src/main/deploy/swerve/swervedrive.json exist.");
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        // swerveDrive.pushOffsetsToEncoders();
    }

    private static final double XBOX_DEADBAND = 0.2;

    private double deadband(double input) {
        if (Math.abs(input) > XBOX_DEADBAND)
            return input;
        else
            return 0;
    }

    // private final TranslationalDrivebase translational = new
    // TranslationalDrivebase() {
    // @Override
    // public void setVelocity(Translation2d velocity) {
    // ChassisSpeeds currentSpeeds = swerveDrive.getRobotVelocity();
    // setVelocities(new ChassisSpeeds(velocity.getX(), velocity.getY(),
    // currentSpeeds.omegaRadiansPerSecond));
    // }

    // @Override
    // public Translation2d getVelocity() {
    // ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
    // return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    // }

    // @Override
    // public Command translationalDrive(CommandXboxController xbox) {
    // return run(() -> {
    // double v_x = deadband(xbox.getLeftX());
    // double v_y = deadband(xbox.getLeftY());
    // // double magnitude = Math.sqrt(v_x * v_x + v_y * v_y);
    // // if (magnitude > TunerConstants.MAX_SPEED) {
    // // v_x *= TunerConstants.MAX_SPEED / magnitude;
    // // v_y *= TunerConstants.MAX_SPEED / magnitude;
    // // }
    // v_x *= TunerConstants.MAX_SPEED;
    // v_y *= TunerConstants.MAX_SPEED;
    // setVelocity(new Translation2d(v_x, v_y));
    // });
    // }

    // // @Override
    // // public Command translationalDrive(CommandXboxController xbox) {
    // // return new TranslationalDriveCommand(translational, xbox.getHID(),
    // // TunerConstants.MAX_SPEED);
    // // }
    // };

    // public TranslationalDrivebase getTranslational() {
    // return translational;
    // }

    // private final RotationalDrivebase rotational = new RotationalDrivebase() {
    // @Override
    // public void setRotationalVelocity(Rotation2d omega) {
    // ChassisSpeeds currentSpeeds = swerveDrive.getRobotVelocity();
    // setVelocities(new ChassisSpeeds(currentSpeeds.vxMetersPerSecond,
    // currentSpeeds.vxMetersPerSecond,
    // omega.getRadians()));
    // }

    // @Override
    // public Rotation2d getRotationalVelocity() {
    // ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
    // return new Rotation2d(speeds.omegaRadiansPerSecond);
    // }

    // @Override
    // public Command rotationalDrive(CommandXboxController xbox) {
    // return run(() -> {
    // setRotationalVelocity(new Rotation2d(TunerConstants.MAX_ANGULAR_SPEED *
    // deadband(xbox.getRightX())));
    // });
    // }

    // // @Override
    // // public Command rotationalDrive(CommandXboxController xbox) {
    // // return new RotationalDriveCommand(rotational, xbox.getHID(),
    // // TunerConstants.MAX_ANGULAR_SPEED);
    // // }
    // };

    // public RotationalDrivebase getRotational() {
    // return rotational;
    // }

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
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    false,
                    false);
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
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

    // public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double
    // headingX, double headingY)
    // {
    // Translation2d scaledInputs = SwerveMath.cubeTranslation(new
    // Translation2d(xInput, yInput));
    // return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
    // scaledInputs.getY(),
    // headingX,
    // headingY,
    // getHeading().getRadians(),
    // Constants.MAX_SPEED);
    // }

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

package frc.robot.subsystems.drive;

import java.io.File;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.RotationalDriveCommand;
import frc.robot.commands.drive.TranslationalDriveCommand;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class YAGSLSwerveDrivetrain implements Drivetrain {
    private static File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    public SwerveDrive swerveDrive;

    public YAGSLSwerveDrivetrain() {
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(TunerConstants.MAX_SPEED);
        } catch(Exception e) {
            System.err.println("Could not create swerve drive. Make sure src/main/deploy/swerve/controllerproperties.json, src/main/deploy/swerve/modules, and src/main/deploy/swerve/swervedrive.json exist.");
        }
    }

    private static final double XBOX_DEADBAND = 0.2;
    private double deadband(double input) {
        if (Math.abs(input) > XBOX_DEADBAND)
            return input;
        else 
            return 0;
    }

    public final TranslationalDrivebase translational = new TranslationalDrivebase() {
        @Override
        public void setVelocity(Translation2d velocity) {
            // swerveDrive.drive(new ChassisSpeeds(velocity.getX(), velocity.getY(), swerveDrive.getRobotVelocity().omegaRadiansPerSecond));
            // swerveDrive.drive(new ChassisSpeeds(velocity.getX(), velocity.getY(), 0));

            
            ChassisSpeeds currentSpeeds = swerveDrive.getRobotVelocity();
            setVelocities(new ChassisSpeeds(velocity.getX(), velocity.getY(), currentSpeeds.omegaRadiansPerSecond));
        }

        @Override
        public Translation2d getVelocity() {
            ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
            return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }
        
        @Override
        public Command translationalDrive(CommandXboxController xbox) {
            return run(() -> {
                double v_x = deadband(xbox.getLeftX());
                double v_y = deadband(xbox.getLeftY());
                // double magnitude = Math.sqrt(v_x * v_x + v_y * v_y);
                // if (magnitude > TunerConstants.MAX_SPEED) {
                //     v_x *= TunerConstants.MAX_SPEED / magnitude;
                //     v_y *= TunerConstants.MAX_SPEED / magnitude;
                // }
                v_x *= TunerConstants.MAX_SPEED;
                v_y *= TunerConstants.MAX_SPEED;
                setVelocity(new Translation2d(v_x, v_y));
            });
        }

        // @Override
        // public Command translationalDrive(CommandXboxController xbox) {
        //     return new TranslationalDriveCommand(translational, xbox.getHID(), TunerConstants.MAX_SPEED);
        // }
    };

    public TranslationalDrivebase getTranslational() {
        return translational;
    }

    public final RotationalDrivebase rotational = new RotationalDrivebase() {
        @Override
        public void setRotationalVelocity(Rotation2d omega) {
            ChassisSpeeds currentSpeeds = swerveDrive.getRobotVelocity();
            setVelocities(new ChassisSpeeds(currentSpeeds.vxMetersPerSecond, currentSpeeds.vxMetersPerSecond, omega.getRadians()));
        }

        @Override
        public Rotation2d getRotationalVelocity() {
            ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
            return new Rotation2d(speeds.omegaRadiansPerSecond);
        }

        @Override
        public Command rotationalDrive(CommandXboxController xbox) {
            return run(() -> {
                setRotationalVelocity(new Rotation2d(TunerConstants.MAX_ANGULAR_SPEED * deadband(xbox.getRightX())));
            });
        }

        // @Override
        // public Command rotationalDrive(CommandXboxController xbox) {
        //     return new RotationalDriveCommand(rotational, xbox.getHID(), TunerConstants.MAX_ANGULAR_SPEED);
        // }
    };

    public RotationalDrivebase getRotational() {
        return rotational;
    }

    public void setVelocities(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    public ChassisSpeeds getVelocities() {
        return swerveDrive.getRobotVelocity();
    }

    public void brake() {
        setVelocities(new ChassisSpeeds(0, 0, 0));
        swerveDrive.setMotorIdleMode(true);
    }
}

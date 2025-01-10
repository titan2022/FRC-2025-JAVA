// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    

    // private final Telemetry logger = new Telemetry(TunerConstants.MAX_SPEED);

    

    private void configureBindings() {
        

        // Drivetrain will execute this command periodically

        //driveController.a().onTrue(drivetrain.runOnce(drivetrain::setFieldControl));
        //driveController.x().onTrue(drivetrain.runOnce(drivetrain::setRobotControl));
        
        
        // driveController.b().whileTrue(drivetrain
        //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));
        // reset the field-centric heading on left bumper press
        //driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        
        // if (Utils.isSimulation()) {
            //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
            // }
            // drivetrain.registerTelemetry(logger::telemeterize);
            
        // robotController.y().whileTrue(intakeCommand);
        // robotController.a().whileTrue(reverseIntakeCommand);
        // Second driver
        // shooter.setDefaultCommand(new ShooterControlCommand(shooter, robotController.getHID(), null));
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
        // return new RunCommand(() -> translationalDrivetrain.setVelocity(new Translation2d(0, 1)));
    }
}
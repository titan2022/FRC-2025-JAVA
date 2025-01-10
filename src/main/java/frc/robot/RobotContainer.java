// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.subsystems.drive.YAGSLSwerveDrivetrain;
import frc.robot.utility.Localizer;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandXboxController driveController = new CommandXboxController(0); // My joystick
    public final CommandXboxController robotController = new CommandXboxController(1); // My joystick

    public final YAGSLSwerveDrivetrain drivetrain = new YAGSLSwerveDrivetrain(); // My drivetrain

    public final Localizer localizer = new Localizer(drivetrain, false, 0);

    public final TranslationalDrivebase translationalDrivetrain = drivetrain.translational;
    public final RotationalDrivebase rotationalDrivebase = drivetrain.rotational;

    // private final Telemetry logger = new Telemetry(TunerConstants.MAX_SPEED);

    private final Command brakeDrivetrain = new RunCommand(() -> {
        drivetrain.brake();
        }, 
        translationalDrivetrain, rotationalDrivebase
    );

    private void configureBindings() {
        translationalDrivetrain.setDefaultCommand(translationalDrivetrain.translationalDrive(driveController, localizer));
        rotationalDrivebase.setDefaultCommand(rotationalDrivebase.rotationalDrive(driveController, localizer));

        // Starting with the implementation of CommandSwerveDrivetrain, field-orientation is handled by the commands and 
        // not by the swerve subsystem.

        // Drivetrain will execute this command periodically

        // driveController.a().onTrue(drivetrain.runOnce(drivetrain::setFieldControl));
        // driveController.x().onTrue(drivetrain.runOnce(drivetrain::setRobotControl));
        // driveController.y().whileTrue(brakeDrivetrain);
        
        // // driveController.b().whileTrue(drivetrain
        // //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));
        // // reset the field-centric heading on left bumper press
        // driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        
        // // if (Utils.isSimulation()) {
        //     //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        //     // }
        //     // drivetrain.registerTelemetry(logger::telemeterize);
            
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
        // return new RunCommand(() -> translationalDrivetrain.setVelocity(new Translation2d(0, 1)));
    }
}

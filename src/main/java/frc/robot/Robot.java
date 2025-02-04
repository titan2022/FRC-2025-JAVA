// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;

import frc.robot.subsystems.drive.TunerConstants;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final CommandXboxController driveController = new CommandXboxController(0); // My driveController
  public final CommandXboxController robotController = new CommandXboxController(1); // My driveController

  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(); // My drivetrain
  // public final TranslationalDrivebase translationalDrivetrain = drivetrain.translational;
  // public final RotationalDrivebase rotationalDrivebase = drivetrain.rotational;

  // private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
  //   .withDeadband(1.0 * 0.1).withRotationalDeadband(15 * Unit.DEG * 0.1) // Add a 10% deadband
  //   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  //   .withSteerRequestType(SteerRequestType.Position);

  private final SwerveRequest.RobotCentric robotCentricDriveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.MAX_SPEED * 0.1).withRotationalDeadband(TunerConstants.MAX_ANGULAR_SPEED * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentric fieldCentricDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MAX_SPEED * 0.1).withRotationalDeadband(TunerConstants.MAX_ANGULAR_SPEED * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  
  private boolean isFieldOriented = false;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
  @Override
  public void robotInit() {
    configureBindings();
  }

  private void configureDriveBindings() {
    // See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java#L53

    if(isFieldOriented) {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              fieldCentricDriveRequest
                  .withVelocityX(-driveController.getLeftY() * TunerConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                  .withVelocityY(-driveController.getLeftX() * TunerConstants.MAX_SPEED) // Drive left with negative X (left)
                  .withRotationalRate(-driveController.getRightX() * TunerConstants.MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
          )
      );
    } else {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              robotCentricDriveRequest
                  .withVelocityX(-driveController.getLeftY() * TunerConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                  .withVelocityY(-driveController.getLeftX() * TunerConstants.MAX_SPEED) // Drive left with negative X (left)
                  .withRotationalRate(-driveController.getRightX() * TunerConstants.MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
          )
      );
    }
  }

  private void toggleFieldOriented() {
    if(isFieldOriented) isFieldOriented = false;
    else isFieldOriented = true;

    configureDriveBindings();
  }

  private void configureBindings() {
    // See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java#L53

    configureDriveBindings();

    driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveController.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
    ));

    // Dpad buttons
    driveController.pov(0).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0.5).withVelocityY(0))
    );
    driveController.pov(90).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(0.5))
    );
    driveController.pov(180).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    );
    driveController.pov(270).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-0.5))
    );

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // toggle field-oriented/robot-oriented on X
    driveController.x().onTrue(drivetrain.runOnce(() -> toggleFieldOriented()));

    // drivetrain.registerTelemetry(logger::telemeterize);
}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    // SmartDashboard.putNumber("curr_velx", translationalDrivetrain.getVelocity().getX());
    // SmartDashboard.putNumber("curr_vely", translationalDrivetrain.getVelocity().getY());
    // SmartDashboard.putNumber("curr_omega", rotationalDrivebase.getRotationalVelocity().getDegrees());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // translationalDrivetrain.removeDefaultCommand();
    // rotationalDrivebase.removeDefaultCommand();
    // m_autonomousCommand = robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    // robotContainer.translationalDrivetrain.setVelocity(new Translation2d(0, 1));

    
  }

  @Override
  public void autonomousPeriodic() {
    // rotationalDrivebase.setRotationalVelocity(new Rotation2d(15 * Unit.DEG));
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    

    // translationalDrivetrain.setDefaultCommand(translationalDrivetrain.translationalDrive(driveController));
    // rotationalDrivebase.setDefaultCommand(rotationalDrivebase.rotationalDrive(driveController));
  }

  @Override
  public void teleopPeriodic() {
    // translationalDrivetrain.setVelocity(new Translation2d(driveController.getLeftX() * 0.5, driveController.getLeftY() * 0.5));
    // rotationalDrivebase.setRotationalVelocity(new Rotation2d(driveController.getRightX() * 0.5));

  //   drivetrain.setControl(
  //     m_driveRequest.withVelocityX(-driveController.getLeftY())
  //        .withVelocityY(-driveController.getLeftX())
  //        .withRotationalRate(-driveController.getRightX())
  //  );
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
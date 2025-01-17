// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.subsystems.drive.YAGSLSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;

public class Robot extends TimedRobot {
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  // private RobotContainer robotContainer;
  
  public final CommandXboxController driveController = new CommandXboxController(0); // My joystick
  public final CommandXboxController robotController = new CommandXboxController(1); // My joystick

  public final YAGSLSwerveDrivetrain drivetrain = new YAGSLSwerveDrivetrain(); // My drivetrain
  // public final TranslationalDrivebase translationalDrivetrain = drivetrain.getTranslational();
  // public final RotationalDrivebase rotationalDrivebase = drivetrain.getRotational();

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
                                                                () -> driveController.getLeftY() * -1,
                                                                () -> driveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driveController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8);
                                                            // .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX,
  driveController::getRightY).headingWhile(false);
 
  private static final double XBOX_DEADBAND = 0.2;
  private double deadband(double input) {
      if (Math.abs(input) > XBOX_DEADBAND)
          return input;
      else 
          return 0;
  }

  @Override
  public void robotInit() {
    // robotContainer = new RobotContainer();

    SmartDashboard.putNumber("angleFactor", SwerveMath.calculateDegreesPerSteeringRotation(21.428571428571427));
    SmartDashboard.putNumber("driveFactor", SwerveMath.calculateMetersPerRotation(4 * Unit.IN, 6.122448979591837));
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
    
  }

  @Override
  public void autonomousPeriodic() {
    // translationalDrivetrain.setVelocity(new Translation2d(1, 0));
    // rotationalDrivebase.setRotationalVelocity(new Rotation2d(1));

    drivetrain.drive(new Translation2d(1, 0), 0, false);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    //translationalDrivetrain.setDefaultCommand(translationalDrivetrain.translationalDrive(driveController));
    //rotationalDrivebase.setDefaultCommand(rotationalDrivebase.rotationalDrive(driveController));

    // Command driveFieldOrientedDirectAngle         = drivetrain. (driveDirectAngle);
    // Command driveFieldOrientedAnglularVelocity    = drivetrain.driveFieldOriented(driveAngularVelocity);

    // drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  @Override
  public void teleopPeriodic() {
    // translationalDrivetrain.setVelocity(new Translation2d(deadband(driveController.getLeftX()) * 0.5, deadband(driveController.getLeftY()) * 0.5));
    // rotationalDrivebase.setRotationalVelocity(new Rotation2d(deadband(driveController.getRightX())));

    // drivetrain.setControl(
    //   m_driveRequest.withVelocityX(-driveController.getLeftY())
    //      .withVelocityY(-driveController.getLeftX())
    //      .withRotationalRate(-driveController.getRightX())
    // );

    
    drivetrain.drive(new Translation2d(0, 1), 0, false);
  }

  @Override
  public void teleopExit() {

  }

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
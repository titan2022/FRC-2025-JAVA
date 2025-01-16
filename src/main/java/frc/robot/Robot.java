// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystemCoral;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.subsystems.drive.YAGSLSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private IntakeSubsystemCoral intake = new IntakeSubsystemCoral();

  public final CommandXboxController driveController = new CommandXboxController(0); // My joystick
  public final CommandXboxController robotController = new CommandXboxController(1); // My joystick

  public final YAGSLSwerveDrivetrain drivetrain = new YAGSLSwerveDrivetrain(); // My drivetrain
  public final TranslationalDrivebase translationalDrivetrain = drivetrain.translational;
  public final RotationalDrivebase rotationalDrivebase = drivetrain.rotational;

  private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
      .withDeadband(1.0 * 0.1).withRotationalDeadband(15 * Unit.DEG * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.Position);

  private static final double XBOX_DEADBAND = 0.15;

  private double deadband(double input) {
    if (Math.abs(input) > XBOX_DEADBAND)
      return input;
    else
      return 0;
  }

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Desired Intake Speed", 0.75);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    translationalDrivetrain.removeDefaultCommand();
    rotationalDrivebase.removeDefaultCommand();
  }

  @Override
  public void autonomousPeriodic() {
    translationalDrivetrain.setVelocity(new Translation2d(0, 1));
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    translationalDrivetrain.setDefaultCommand(translationalDrivetrain.translationalDrive(driveController));
    rotationalDrivebase.setDefaultCommand(rotationalDrivebase.rotationalDrive(driveController));
  }

  @Override
  public void teleopPeriodic() {

    if (robotController.y().getAsBoolean()) { // using the y button as set intake speed
      intake.setWheelSpeed(SmartDashboard.getNumber("Desired Intake Speed", 0));
    } else if (robotController.a().getAsBoolean()) // using the a button as stop intake
      intake.stop();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
  //from the old RobotContainer.java
  private void configureBindings() {
    // Drivetrain will execute this command periodically
  }

  public void RobotContainer() {
        configureBindings();
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
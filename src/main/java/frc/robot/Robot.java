// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.subsystems.drive.YAGSLSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer robotContainer;

  public final CommandXboxController driveController = new CommandXboxController(0); // My joystick
  public final CommandXboxController robotController = new CommandXboxController(1); // My joystick

  public final YAGSLSwerveDrivetrain drivetrain = new YAGSLSwerveDrivetrain(); // My drivetrain
  public final TranslationalDrivebase translationalDrivetrain = drivetrain.translational;
  public final RotationalDrivebase rotationalDrivebase = drivetrain.rotational;

  private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
      .withDeadband(1.0 * 0.1).withRotationalDeadband(15 * Unit.DEG * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.Position);

  @AutoLogOutput
  private Pose2d currentPose = new Pose2d();
  private Translation2d direction;
  private double speed;
  private PoseLogger2d poseLogger;
  private double period; 

  public Robot() {
    Logger.recordMetadata("PoseEstimator", "I hope this works");

    if (isReal()) {
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      System.out.println("Logging to: " + logPath);
    }

    Logger.registerURCL(URCL.startExternal());
    System.out.println("AdvantageKit Logger starting...");
    Logger.start();
  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    direction = new Translation2d(0.1, 0.1);
    period = 0.02;
    speed = 0.1;

    double magnitude = direction.getNorm();
    this.direction = (magnitude != 0) ? direction.div(magnitude) : new Translation2d(0, 0);

  }

  @Override
  public void robotPeriodic() {
    Translation2d displacement = direction.times(speed * period);

    currentPose = currentPose.plus(new Transform2d(displacement, new Rotation2d()));


    Logger.recordOutput("currentPose", currentPose);
    CommandScheduler.getInstance().run();

    // SmartDashboard.putNumber("curr_velx",
    // translationalDrivetrain.getVelocity().getX());
    // SmartDashboard.putNumber("curr_vely",
    // translationalDrivetrain.getVelocity().getY());
    // SmartDashboard.putNumber("curr_omega",
    // rotationalDrivebase.getRotationalVelocity().getDegrees());
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
    // translationalDrivetrain.removeDefaultCommand();
    // rotationalDrivebase.removeDefaultCommand();
    // m_autonomousCommand = robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
    // robotContainer.translationalDrivetrain.setVelocity(new Translation2d(0, 1));

  }

  @Override
  public void autonomousPeriodic() {
    // rotationalDrivebase.setRotationalVelocity(new Rotation2d(15 * Unit.DEG));
  }

  @Override
  public void autonomousExit() {
  }

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
    translationalDrivetrain
        .setVelocity(new Translation2d(driveController.getLeftX() * 0.5, driveController.getLeftY() * 0.5));
    rotationalDrivebase.setRotationalVelocity(new Rotation2d(driveController.getRightX() * 0.5));

    // drivetrain.setControl(
    // m_driveRequest.withVelocityX(-driveController.getLeftY())
    // .withVelocityY(-driveController.getLeftX())
    // .withRotationalRate(-driveController.getRightX())
    // );
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
}
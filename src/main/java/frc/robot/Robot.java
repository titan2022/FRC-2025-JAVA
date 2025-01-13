// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer robotContainer;
  
  public final CommandXboxController driveController = new CommandXboxController(0); // My joystick
    public final CommandXboxController robotController = new CommandXboxController(1); // My joystick

    public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(); // My drivetrain
    // public final TranslationalDrivebase translationalDrivetrain = drivetrain.translational;
    // public final RotationalDrivebase rotationalDrivebase = drivetrain.rotational;

    private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
    .withDeadband(1.0 * 0.1).withRotationalDeadband(15 * Unit.DEG * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position);
 
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
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

    drivetrain.setControl(
      m_driveRequest.withVelocityX(-driveController.getLeftY())
         .withVelocityY(-driveController.getLeftX())
         .withRotationalRate(-driveController.getRightX())
   );
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
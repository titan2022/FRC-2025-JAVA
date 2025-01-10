// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;

public class Robot extends TimedRobot {
  private final XboxController xbox = new XboxController(0);
  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(); // My drivetrain

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {
    
    // drivetrain.translational.setVelocity(new Translation2d(0, 1));
    drivetrain.rotational.setRotationalVelocity(new Rotation2d(5 * Unit.DEG));
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.translational.setVelocity(new Translation2d(xbox.getLeftX() * 0.5, xbox.getLeftY() * 0.5));
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

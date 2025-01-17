// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.YAGSLSwerveDrivetrain;
import frc.robot.utility.Constants.Unit;
import swervelib.math.SwerveMath;

public class Robot extends TimedRobot {
	public final CommandXboxController driveController = new CommandXboxController(0);

	public final YAGSLSwerveDrivetrain drivetrain = new YAGSLSwerveDrivetrain();

	@Override
	public void robotInit() {
		SmartDashboard.putNumber("angleFactor", SwerveMath.calculateDegreesPerSteeringRotation(21.428571428571427));
		SmartDashboard.putNumber("driveFactor", SwerveMath.calculateMetersPerRotation(4 * Unit.IN, 6.122448979591837));
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

	}

	@Override
	public void autonomousPeriodic() {
		drivetrain.drive(new Translation2d(1, 0), 0, false);
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		// translationalDrivetrain.setDefaultCommand(translationalDrivetrain.translationalDrive(driveController));
		// rotationalDrivebase.setDefaultCommand(rotationalDrivebase.rotationalDrive(driveController));
	}

	@Override
	public void teleopPeriodic() {
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
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
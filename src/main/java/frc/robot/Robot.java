// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.drive.DrivingCommand;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralScorerSubsystem;
import frc.robot.subsystems.DealgifierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevationTarget;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final CommandXboxController driveController = new CommandXboxController(0); // My driveController
  public final CommandXboxController robotController = new CommandXboxController(1); // My driveController

  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(); // My drivetrain
 
  private DrivingCommand drivingCommand = new DrivingCommand(drivetrain, driveController);

  // Create auto chooser using all the autos in the project
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  // Subsystems
  private final CoralScorerSubsystem coralScorer = new CoralScorerSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final DealgifierSubsystem dealgifierSubsystem = new DealgifierSubsystem();

  // private final Localizers localizers = new Localizers(
  //   new OdometryLocalizer(drivetrain), 
  //   new TitanProcessingLocalizer(5800)
  // );

  @Override
  public void robotInit() {
    setBindings();
    setUpAutos();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // localizers.step();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  public void setBindings() {
    drivetrain.setDefaultCommand(drivingCommand);

    // Coral scorer controls
    robotController.rightBumper().whileTrue(
      coralScorer.timedScoreCoralCommand(false)
    );

    // Backwards coral scoring
    robotController.b().whileTrue(
      coralScorer.timedScoreCoralCommand(true)
    );

    // Elevator controls
    // Left dpad is elevate to coral intake level
    robotController.pov(270).whileTrue (elevator.elevateCommand(ElevationTarget.CoralIntake));
    robotController.pov(180).whileTrue(elevator.elevateCommand(ElevationTarget.L1));
    robotController.pov(90).whileTrue(elevator.elevateCommand(ElevationTarget.L2));
    robotController.pov(0).whileTrue(elevator.elevateCommand(ElevationTarget.L3));

    // X any Y is elevate to remove algae levels
    robotController.x().whileTrue(elevator.elevateCommand(ElevationTarget.AlgaeL2));
    robotController.y().whileTrue(elevator.elevateCommand(ElevationTarget.AlgaeL3));
    
    elevator.setDefaultCommand(elevator.manualElevationCommand(robotController));

    // Coral intake controls

    // Elevate down to the coral intake level,
    // then run the coral intake and scorer motors to move the coral in.
    robotController.leftBumper().whileTrue(
      // elevator.elevateCommand(ElevationTarget.CoralIntake)
      // .andThen(
        new CoralIntakeCommand(coralIntake, coralScorer)
      // )
    );

    // Dealgifier controls
    robotController.a().whileTrue(dealgifierSubsystem.dealgifyCommand());
  }

  public void setUpAutos() {
    // Register named commands
    NamedCommands.registerCommand("Elevate to intake level", elevator.elevateCommand(ElevationTarget.CoralIntake));
    NamedCommands.registerCommand("Elevate L1", elevator.elevateCommand(ElevationTarget.L1));
    NamedCommands.registerCommand("Elevate L2", elevator.elevateCommand(ElevationTarget.L2));
    NamedCommands.registerCommand("Elevate L3", elevator.elevateCommand(ElevationTarget.L3));
    NamedCommands.registerCommand("Elevate Algae L2", elevator.elevateCommand(ElevationTarget.AlgaeL2));
    NamedCommands.registerCommand("Elevate Algae L3", elevator.elevateCommand(ElevationTarget.AlgaeL3));

    // TODO: Figure out how to finish elevating before ending the command
    NamedCommands.registerCommand("Intake coral", 
      elevator.elevateCommand(ElevationTarget.CoralIntake)
      .andThen(
        new CoralIntakeCommand(coralIntake, coralScorer)
      )
    );

    NamedCommands.registerCommand("Score coral", coralScorer.timedScoreCoralCommand(false));

    NamedCommands.registerCommand("Reef left align", Commands.print("Warning: reef align is not implemented!"));
    NamedCommands.registerCommand("Reef right align", Commands.print("Warning: reef align is not implemented!"));

    // Use event markers as triggers
    // new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

    // Add the auto chooser to the SmartDashboard so we can select the auto from the dropdown
    SmartDashboard.putData(autoChooser);

    // Set the default option for autoChooser
    // autoChooser.setDefaultOption();
  }

  public Command getAutonomousCommand() {
    return null;//autoChooser.getSelected();
  }

  @Override
  public void autonomousInit() {
    // See https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/Robot.java#L58
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // Quick fix
    //elevator.resetTarget();
  }

  @Override
  public void teleopPeriodic() {
    // translationalDrivetrain.setVelocity(new Translation2d(driveController.getLeftX() * 0.5, driveController.getLeftY() * 0.5));
    // rotationalDrivebase.setRotationalVelocity(new Rotation2d(driveController.getRightX() * 0.5));

    // drivetrain.setControl(
    //   robotCentricDriveRequest.withVelocityX(-driveController.getLeftY())
    //      .withVelocityY(-driveController.getLeftX())
    //      .withRotationalRate(-driveController.getRightX())
   	// );

	
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
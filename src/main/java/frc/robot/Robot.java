// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.drive.DrivingCommand;
import frc.robot.commands.drive.NaiveDriveToPoseCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AngleTarget;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralScorerSubsystem;
import frc.robot.subsystems.DealgifierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevationTarget;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utility.Constants;
import frc.robot.utility.Localizers;
import frc.robot.utility.OdometryLocalizer;
import frc.robot.utility.ReefLocations;
import frc.robot.utility.TitanProcessingLocalizer;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final CommandXboxController driveController = new CommandXboxController(0); // My driveController
  public final CommandXboxController robotController = new CommandXboxController(1); // My driveController

  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(); // My drivetrain
 
  private DrivingCommand drivingCommand = new DrivingCommand(drivetrain, driveController);

  // Create auto chooser using all the autos in the project
  private SendableChooser<Command> autoChooser;

  // Subsystems
  private final CoralScorerSubsystem coralScorer = new CoralScorerSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final DealgifierSubsystem dealgifier = new DealgifierSubsystem();
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  private final Localizers localizers = new Localizers(
    new OdometryLocalizer(drivetrain), 
    new TitanProcessingLocalizer(5804)
  );

  @Override
  public void robotInit() {
    setBindings();
    setUpAutos();
    //SignalLogger.setPath("/media/sda1/");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    localizers.step();

    SmartDashboard.putNumber("x-distance to tag 17", localizers.getVision().getMeasurement().pose.getX() - Constants.tagLayout.getTagPose(17).get().getX());
    SmartDashboard.putNumber("y-distance to tag 17", localizers.getVision().getMeasurement().pose.getY() - Constants.tagLayout.getTagPose(17).get().getY());
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

    // Shifting coral forward when elevator passes bumper
    //coralScorer.setDefaultCommand(coralScorer.coralShiftingCommand(elevator));

    // Elevator controls
    // Left dpad is elevate to coral intake level
    robotController.pov(270).whileTrue (elevator.elevateCommand(ElevationTarget.CoralIntake));
    robotController.pov(270).whileTrue (elevator.elevateCommand(ElevationTarget.CoralIntake));
    robotController.pov(180).whileTrue(elevator.elevateCommand(ElevationTarget.L1));
    robotController.pov(90).whileTrue(elevator.elevateCommand(ElevationTarget.L2));
    robotController.pov(0).whileTrue(elevator.elevateCommand(ElevationTarget.L3));

    // X any Y is elevate to remove algae levels
    robotController.x().whileTrue(
      elevator.elevateCommand(ElevationTarget.AlgaeL2)
      .alongWith(dealgifier.dealgifyCommand())
    );
    robotController.y().whileTrue(
      elevator.elevateCommand(ElevationTarget.AlgaeL3)
      .alongWith(dealgifier.dealgifyCommand())
    );
    
    elevator.setDefaultCommand(elevator.manualElevationCommand(robotController));

    // Coral intake controls

    // Elevate down to the coral intake level,
    // then run the coral intake and scorer motors to move the coral in.
    robotController.leftBumper().whileTrue(
        elevator.elevateCommand(ElevationTarget.CoralIntake)
        .andThen(
        new CoralIntakeCommand(coralIntake, coralScorer)
       )
    );

    // Dealgifier controls
    robotController.a().whileTrue(dealgifier.dealgifyCommand());

    // Auto align
    driveController.leftTrigger().whileTrue(NaiveDriveToPoseCommand.driveToNearestScoringLocation(drivetrain, localizers.getOdometry()));
    // driveController.rightTrigger().whileTrue(NaiveDriveToPoseCommand.driveToNearestRightScoringLocation(drivetrain, localizers.getVision()));
    driveController.rightTrigger().whileTrue(CommandSwerveDrivetrain.driveToNearestScoringLocation(localizers.getOdometry()));
    //Algae Intake Controls
    robotController.rightTrigger  ().whileTrue(
      algaeIntakeSubsystem.intakeCommand()
    );
    robotController.leftTrigger().whileTrue(
      algaeIntakeSubsystem.scoreCommand()
    );
  }

  public void setUpAutos() {
    // Register named commands
    NamedCommands.registerCommand("Elevate to intake level", elevator.elevateCommand(ElevationTarget.CoralIntake));
    NamedCommands.registerCommand("Elevate L1", elevator.elevateCommand(ElevationTarget.L1));
    NamedCommands.registerCommand("Elevate L2", 
    elevator.elevateCommand(ElevationTarget.L2)
    //.alongWith(coralScorer.coralShiftingCommand(elevator))
    );
    NamedCommands.registerCommand("Elevate L3", elevator.elevateCommand(ElevationTarget.L3)
    //.alongWith(coralScorer.coralShiftingCommand(elevator))
    );
    NamedCommands.registerCommand("Dealgify L2", 
      elevator.elevateCommand(ElevationTarget.AlgaeL2)
      .alongWith(dealgifier.dealgifyCommand()).withTimeout(.75)
    );
    NamedCommands.registerCommand("Dealgify L3", 
      elevator.elevateCommand(ElevationTarget.AlgaeL3)
      .alongWith(dealgifier.dealgifyCommand()).withTimeout(.75)
    );

    // TODO: Figure out how to finish elevating before ending the command
    NamedCommands.registerCommand("Intake coral", 
      elevator.elevateCommand(ElevationTarget.CoralIntake)
      .andThen(
        new CoralIntakeCommand(coralIntake, coralScorer)
      )
    );

    NamedCommands.registerCommand("Score coral", coralScorer.timedScoreCoralCommand(false));

    //NamedCommands.registerCommand("Reef left align", Commands.print("Warning: reef align is not implemented!"));
    NamedCommands.registerCommand("Reef right align", Commands.print("Warning: reef align is not implemented!"));

    // Use event markers as triggers
    // new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
    autoChooser = AutoBuilder.buildAutoChooser();
    // Add the auto chooser to the SmartDashboard so we can select the auto from the dropdown
    SmartDashboard.putData(autoChooser);

    // Set the default option for autoChooser
    // autoChooser.setDefaultOption();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
    
    //SignalLogger.start();
    // Quick fix
    //elevator.resetTarget();
    //elevator.resetTarget();
    
    localizers.enableMixing();
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
  public void teleopExit() {
    //SignalLogger.stop();
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
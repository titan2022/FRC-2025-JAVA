package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralScorerSubsystem;

/**
 * Intakes coral into the coral scorer.
 * Runs both the coral intake and scorer.
 * The elevator should already be at the coral intake level.
 */
public class CoralIntakeCommand extends Command {
  private final CoralIntakeSubsystem intake;
  private final CoralScorerSubsystem scorer;

  public CoralIntakeCommand(CoralIntakeSubsystem intake, CoralScorerSubsystem scorer) {
    this.intake = intake;
    this.scorer = scorer;
    addRequirements(intake, scorer);
  }

  @Override
  public void initialize() {
    intake.startIntaking();
    scorer.moveCoral();
  }

  @Override
  public boolean isFinished() {
    return scorer.canSeeCoral();
  }
}

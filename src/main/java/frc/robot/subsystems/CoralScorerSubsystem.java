package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants.Unit;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class CoralScorerSubsystem extends SubsystemBase {
  // TODO: Specify CAN IDs
  private static final TalonFX scoringMotor = new TalonFX(0, "CANivore");
  private static final Canandcolor canandcolor = new Canandcolor(0);

  // TODO: Measure this
  private static final double PROXIMITY_THRESHOLD = 0.5;

  // TODO: Determine speed
  private static final double MOVE_CORAL_SPEED = 0.5; // out of 1.0
  private static final double SCORE_CORAL_SPEED = 1.0; // out of 1.0

  private static final long SCORE_CORAL_TIMEOUT = 1 * 1000000; // microseconds

  public CoralScorerSubsystem() {
    // Brake the scoring motor while not in use
    scoringMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(
      runOnce(
        () -> {
          stopMovingCoral();
        }
      )
      .andThen(run(() -> {}))
      .withName("Idle")
    );
  }

  public boolean canContactCanandcolor() {
    return canandcolor.isConnected();
  }

  public boolean canSeeCoral() {
    return canandcolor.getProximity() > PROXIMITY_THRESHOLD;
  }

  /** Start moving coral into the scorer
   */
  public void moveCoral() {
    scoringMotor.set(MOVE_CORAL_SPEED);
  }

  /** Start scoring coral into the scorer
   */
  public void scoreCoral() {
    scoringMotor.set(SCORE_CORAL_SPEED);
  }

  private class TimedScoreCoralCommand extends Command {
    private long startTime; // relative to RobotController.getFPGATime()
    private CoralScorerSubsystem coralScorer;

    public TimedScoreCoralCommand(CoralScorerSubsystem coralScorer) {
      this.coralScorer = coralScorer;
      addRequirements(coralScorer);
    }

    @Override // Called at beginning of command
    public void initialize() {
      startTime = RobotController.getFPGATime();

      scoreCoral();
    }

    @Override
    public boolean isFinished() {
      return RobotController.getFPGATime() >= startTime + SCORE_CORAL_TIMEOUT;
    }
  }

  /** Command that scores coral for a specific time.
   */
  public Command timedScoreCoralCommand() {
    return new TimedScoreCoralCommand(this);
  }

  /** Move coral back into the elevator
   */
  public void reverseMoveCoral() {
    scoringMotor.set(-MOVE_CORAL_SPEED);
  }
  
  /** Stop moving coral
   */
  public void stopMovingCoral() {
    scoringMotor.stopMotor();
  }
}

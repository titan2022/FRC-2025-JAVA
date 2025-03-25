package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralScorerSubsystem extends SubsystemBase {
  // TODO: Specify CAN IDs
  private static final TalonFX scoringMotor = new TalonFX(42, "rio");
  private static final Canandcolor canandcolor = new Canandcolor(0);

  private static final double PROXIMITY_THRESHOLD = 0.020;

  // TODO: Determine speed
  private static final double MOVE_CORAL_SPEED = 1.8; // in volts
  private static final double SCORE_CORAL_SPEED = 1.5; // in volts

  private static final long SCORE_CORAL_TIMEOUT = 1 * 1000000; // microseconds
  private static final long INDEX_CORAL_TIMEOUT = 1 * 150000; // microseconds

  private long coralVisibleStart;
  private boolean sawCoralInLastFrame;

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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Canandcolor proximity", canandcolor.getProximity());
  }

  public boolean canContactCanandcolor() {
    return canandcolor.isConnected();
  }

  public boolean canSeeCoral() {
    return canandcolor.getProximity() < PROXIMITY_THRESHOLD;
  }

  //RobotController.getFPGATime()
  public boolean coralFinishedIndexing() {
    if(canSeeCoral()) {
      long currentTime = RobotController.getFPGATime();
      if (!sawCoralInLastFrame) {
        coralVisibleStart = currentTime;
      }

      if (currentTime - coralVisibleStart >= INDEX_CORAL_TIMEOUT) {
        return true;
      }

      sawCoralInLastFrame = true;
    } else {
      sawCoralInLastFrame = false;
    }

    return false;
  }

  /** Start moving coral into the scorer
   */
  public void moveCoral() {
    scoringMotor.setVoltage(MOVE_CORAL_SPEED);
  }

  /** Start scoring coral into the scorer
   */
  public void scoreCoral(boolean isReversed) {
    scoringMotor.setVoltage(SCORE_CORAL_SPEED * (isReversed ? -0.75 : 1));
  }

  private class TimedScoreCoralCommand extends Command {
    private long startTime; // relative to RobotController.getFPGATime()
    private CoralScorerSubsystem coralScorer;
    private boolean isReversed = false;

    public TimedScoreCoralCommand(CoralScorerSubsystem coralScorer, boolean isReversed) {
      this.coralScorer = coralScorer;
      this.isReversed = isReversed;
      addRequirements(coralScorer);
    }

    @Override // Called at beginning of command
    public void initialize() {
      startTime = RobotController.getFPGATime();

      scoreCoral(isReversed);
    }

    @Override
    public boolean isFinished() {
      return false;//RobotController.getFPGATime() >= startTime + SCORE_CORAL_TIMEOUT;
    }
  }

  /** Command that scores coral for a specific time.
   */
  public Command timedScoreCoralCommand(boolean isReversed) {
    return new TimedScoreCoralCommand(this, isReversed);
  }

  /** Move coral back into the elevator
   */
  public void reverseMoveCoral() {
    scoringMotor.setVoltage(-MOVE_CORAL_SPEED);
  }
  
  /** Stop moving coral
   */
  public void stopMovingCoral() {
    scoringMotor.stopMotor();
  }

  /** 
   * @return whether a coral was detected in the last frame
   */
  public boolean getSawCoralInLastFrame() {
    return sawCoralInLastFrame;
  }
}

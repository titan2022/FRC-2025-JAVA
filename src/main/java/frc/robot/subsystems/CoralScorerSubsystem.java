package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralScorerSubsystem extends SubsystemBase {
  private static final double PROXIMITY_THRESHOLD = 0.021;
  private static final double MOVE_CORAL_SPEED = 1.8; // in volts
  private static final double SCORE_CORAL_SPEED = 1.5; // in volts
  private static final long SCORE_CORAL_TIMEOUT = 1 * 1000000; // microseconds
  private static final long INDEX_CORAL_TIMEOUT = 1 * 1500; // microseconds
  private static final long SHIFT_FORWARD_CORAL_TIMEOUT = 1 * 30000;

  private static final long SHIFT_BACKWARD_CORAL_TIMEOUT = 1 * 48000;

  // TODO: determine this
  private static final double CORAL_SHIFT_ELEVATOR_HEIGHT_INCHES = 4.0;
  
  private static final TalonFX scoringMotor = new TalonFX(42, "rio");
  private static final Canandcolor canandcolor = new Canandcolor(0);

  private long coralVisibleStart;
  private double elevatorHeight = 0;
  private boolean sawCoralInLastFrame;
  private boolean coralShifted = false; // For shifting coral forward after elevator is above the bumper
  private boolean coralMoving = false; // Ignore shifting if it's being moved
  private boolean coralMovedAfterShift = false; // Don't shift back if user already shifted the coral

  public CoralScorerSubsystem() {

    // Brake the scoring motor while not in use
    scoringMotor.setNeutralMode(NeutralModeValue.Brake);

    // Why was this here?
    // setDefaultCommand(
    //   runOnce(
    //     () -> {
    //       stopMovingCoral();
    //     }
    //   )
    //   .andThen(run(() -> {}))
    //   .withName("Idle")
    // );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Canandcolor proximity", canandcolor.getProximity());
    SmartDashboard.putBoolean("Coral shifted", coralShifted);
  }

  public void resetCoralShifting() {
    coralMovedAfterShift = false;
    coralShifted = false;
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
    coralMoving = true;
    if (coralShifted) {
      coralMovedAfterShift = true;
    }
    scoringMotor.setVoltage(MOVE_CORAL_SPEED);
  }

  /** Start scoring coral into the scorer
   */
  public void scoreCoral(boolean isReversed) {
    coralMoving = true;
    if (coralShifted) {
      coralMovedAfterShift = true;
    }
    scoringMotor.setVoltage(SCORE_CORAL_SPEED * (isReversed ? -0.75 : 1));
  }

  private class TimedScoreCoralCommand extends Command {
    private long startTime; // relative to RobotController.getFPGATime()
    private CoralScorerSubsystem coralScorer;
    private boolean isReversed = false;
    private long time = SCORE_CORAL_TIMEOUT;

    public TimedScoreCoralCommand(CoralScorerSubsystem coralScorer, boolean isReversed) {
      this.coralScorer = coralScorer;
      this.isReversed = isReversed;
      addRequirements(coralScorer);
    }

    public TimedScoreCoralCommand(CoralScorerSubsystem coralScorer, boolean isReversed, long time) {
      this.coralScorer = coralScorer;
      this.isReversed = isReversed;
      this.time = time;
      addRequirements(coralScorer);
    }

    @Override // Called at beginning of command
    public void initialize() {
      startTime = RobotController.getFPGATime();

      scoreCoral(isReversed);
    }

    @Override
    public void end(boolean isInterrupted) {
      stopMovingCoral();
    }

    @Override
    public boolean isFinished() {
      return RobotController.getFPGATime() >= startTime + time;
    }
  }

  private class CoralShiftingCommand extends Command {
    private CoralScorerSubsystem coralScorer;
    private ElevatorSubsystem elevator;

    public CoralShiftingCommand(CoralScorerSubsystem coralScorer, ElevatorSubsystem elevator) {
      this.coralScorer = coralScorer;
      this.elevator = elevator;

      addRequirements(coralScorer);
    }

    @Override
    public void initialize() {
      new Trigger(() -> !coralMoving && elevatorHeight >= CORAL_SHIFT_ELEVATOR_HEIGHT_INCHES && !coralShifted).onTrue(
        new SequentialCommandGroup(
          Commands.runOnce(() -> {coralShifted = true;}),
          timedScoreCoralCommand(false,SHIFT_FORWARD_CORAL_TIMEOUT)
        )
      );
      
      new Trigger(() -> !coralMoving && elevatorHeight < CORAL_SHIFT_ELEVATOR_HEIGHT_INCHES && coralShifted).onTrue(
        new SequentialCommandGroup(
          timedScoreCoralCommand(true,(long)(SHIFT_BACKWARD_CORAL_TIMEOUT)),
          Commands.runOnce(() -> {coralShifted = false;})
        )
      );
    }

    @Override
    public void execute() {
      elevatorHeight = elevator.getElevatorPosition();
      if (elevatorHeight < CORAL_SHIFT_ELEVATOR_HEIGHT_INCHES) {
        //resetCoralShifting();
      }
    }

    @Override
    public void end(boolean isInterrupted) {

    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  /** Command that scores coral for a specific time.
   */
  public Command timedScoreCoralCommand(boolean isReversed) {
    return new TimedScoreCoralCommand(this, isReversed);
  }

  /** Command that scores coral for a specified time.
   */
  public Command timedScoreCoralCommand(boolean isReversed, long time) {
    return new TimedScoreCoralCommand(this, isReversed, time);
  }

  /*
   * Command that shifts the coral after the elevator passes the bumper.
   */
  public Command coralShiftingCommand(ElevatorSubsystem elevator) {
    return new CoralShiftingCommand(this, elevator);
  }

  /** Move coral back into the elevator
   */
  public void reverseMoveCoral() {
    coralMoving = true;
    if (coralShifted) {
      coralMovedAfterShift = true;
    }
    scoringMotor.setVoltage(-MOVE_CORAL_SPEED);
  }
  
  /** Stop moving coral
   */
  public void stopMovingCoral() {
    coralMoving = false;
    scoringMotor.setVoltage(0);
    //scoringMotor.stopMotor();
  }
}

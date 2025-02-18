package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class CoralScorerSubsystem extends SubsystemBase {
  // TODO: Specify CAN IDs
  private static final TalonFX scoringMotor = new TalonFX(0, "CANivore");
  private static final TalonFX dealgifierMotor = new TalonFX(0, "CANivore");
  private static final Canandcolor canandcolor = new Canandcolor(0);

  // TODO: Measure this
  private static final double PROXIMITY_THRESHOLD = 0.5;

  // TODO: Determine speed
  private static final double MOVE_CORAL_SPEED = 0.5; // out of 1.0
  private static final double SCORE_CORAL_SPEED = 1.0; // out of 1.0
  private static final double DEALGIFY_SPEED = 0.5; // out of 1.0

  public CoralScorerSubsystem() {
    // Brake the scoring motor while not in use
    scoringMotor.setNeutralMode(NeutralModeValue.Brake);
    // Allow the dealgifier to coast while not in use
    dealgifierMotor.setNeutralMode(NeutralModeValue.Coast);
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

  /** Start dealgifying
   */
  public void startDealgifying() {
    dealgifierMotor.set(DEALGIFY_SPEED);
  }

  /** Stop dealgifying
   */
  public void stopDealgifying() {
    dealgifierMotor.stopMotor();
  }
}

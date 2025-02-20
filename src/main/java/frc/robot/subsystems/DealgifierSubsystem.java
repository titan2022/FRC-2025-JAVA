package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DealgifierSubsystem extends SubsystemBase {
  // TODO: Specify CAN IDs
  private static final TalonFX dealgifierMotor = new TalonFX(0, "CANivore");

  // TODO: Determine this
  private static final double DEALGIFY_SPEED = 4; // in volts

  public DealgifierSubsystem() {
    // Allow the dealgifier to coast while not in use
    dealgifierMotor.setNeutralMode(NeutralModeValue.Coast);

    setDefaultCommand(
      runOnce(
        () -> {
          stopDealgifying();
        }
      )
      .andThen(run(() -> {}))
      .withName("Idle")
    );
  }

  /** Start dealgifying
   */
  public void startDealgifying() {
    dealgifierMotor.setVoltage(DEALGIFY_SPEED);
  }

  /** Stop dealgifying
   */
  public void stopDealgifying() {
    dealgifierMotor.stopMotor();
  }

  /** Dealgify while this command is running
   */
  public Command dealgifyCommand() {
    return startEnd(
      () -> startDealgifying(),
      () -> stopDealgifying()
    );
  }
}

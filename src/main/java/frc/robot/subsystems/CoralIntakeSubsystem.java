package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {
  // TODO: Specify CAN IDs
  private static final TalonFX intakeMotor = new TalonFX(60, "rio");

  // TODO: Figure out this speed
  private static final double INTAKE_SPEED = -3; // in volts

  public boolean isIntaking = false;

  public CoralIntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(
      runOnce(
        () -> {
          stopIntaking();
        }
      )
      .andThen(run(() -> {}))
      .withName("Idle")
    );
  }

  public void startIntaking() {
    intakeMotor.setVoltage(INTAKE_SPEED);
    isIntaking = true;
  }

  public void stopIntaking() {
    intakeMotor.stopMotor();
    isIntaking = false;
  }

  public boolean isIntaking() {
    return isIntaking;
  }
}

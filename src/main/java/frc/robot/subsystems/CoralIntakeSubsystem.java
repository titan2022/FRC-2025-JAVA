package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {
  // TODO: Specify CAN IDs
  private static final TalonFX intakeMotor = new TalonFX(0, "CANivore");

  // TODO: Figure out this speed
  private static final double INTAKE_SPEED = 0.5; // out of 1.0 = max output

  public CoralIntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(run(
      () -> {
        stopIntaking();
      }
    ));
  }

  public void startIntaking() {
    intakeMotor.set(INTAKE_SPEED);
  }

  public void stopIntaking() {
    intakeMotor.stopMotor();
  }
}

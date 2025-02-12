package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  private static final TalonFX leftMotor = new TalonFX(0, "CANivore");
  private static final TalonFX rightMotor = new TalonFX(0, "CANivore");
  private static final PIDController pid = new PIDController(1.0, 0.0, 0.0);

  private double initialLeft = 0.0;
  private double initialRight = 0.0;

  public ElevatorSubsystem() {
    // Brake the motors while not elevating
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    // TODO: Use a non-deprecated method
    // Invert the right motor
    rightMotor.setInverted(false);

    // Set default command to turn off the left and right motors, and then idle
    setDefaultCommand(
      runOnce(
          () -> {
            leftMotor.disable();
            rightMotor.disable();
          })
        .andThen(run(() -> {}))
        .withName("Idle"));

    resetMeasurement();
  }

  public void resetMeasurement() {
    // FIXME
  }

  public double getMeasurement() {
    // FIXME
    return 0.0;
  }

  public Command elevateCommand(double target) {
    return run(
        () -> {
          leftMotor.set(
            pid.calculate(target, getMeasurement())
          );
          rightMotor.set(
            pid.calculate(target, getMeasurement())
          );
        })
      .withName("Elevate");
  }
}

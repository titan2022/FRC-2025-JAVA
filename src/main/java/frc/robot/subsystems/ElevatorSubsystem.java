package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  private static final TalonFX leftMotor = new TalonFX(0, "CANivore");
  private static final TalonFX rightMotor = new TalonFX(0, "CANivore");
  private static final PIDController pid = new PIDController(1.0, 0.0, 0.0);

  private double initialLeft = 0.0;
  private double initialRight = 0.0;

  private StatusSignal<Angle> leftPositionStatusSignal;
  private StatusSignal<Angle> rightPositionStatusSignal;

  // TODO: Figure out good values for these constants
  // The unit is rotations of the motor (rotor, excluding gear ratio)
  private static double POSITION_DEADBAND = 0.05;
  private static double MANUAL_ELEVATION_VELOCITY = 0.5;
  private static double MANUAL_ELEVATION_DEADBAND = 0.15;

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

    leftPositionStatusSignal = leftMotor.getRotorPosition();
    rightPositionStatusSignal = rightMotor.getRotorPosition();

    resetMeasurement();
  }

  public void resetMeasurement() {
    leftPositionStatusSignal.refresh();
    initialLeft = leftPositionStatusSignal.getValueAsDouble();
    rightPositionStatusSignal.refresh();
    initialRight = rightPositionStatusSignal.getValueAsDouble();
  }

  public double getMeasurement() {
    double left = leftPositionStatusSignal.getValueAsDouble();
    double right = rightPositionStatusSignal.getValueAsDouble();
    double leftMeasurement = left - initialLeft;
    double rightMeasurement = right - initialRight;
    return (leftMeasurement + rightMeasurement)/2;
  }

  public enum ElevationTarget {
    // TODO: Get the real values of these
    CoralIntake(0),
    L1(10),
    L2(20),
    L3(30),
    ;

    private double targetValue;
    private ElevationTarget(double targetValue) {
      this.targetValue = targetValue;
    }
    public double getValue() {
      return targetValue;
    }
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
      }
    ).until(
      () -> Math.abs(getMeasurement() - target) < POSITION_DEADBAND
    ).withName("Elevate to target");
  }

  public Command elevateCommand(ElevationTarget target) {
    return elevateCommand(target.getValue());
  }

  public Command applyVelocityCommand(double velocity) {
    return runOnce(
      () -> {
        leftMotor.set(velocity);
        rightMotor.set(velocity);
      }
    ).withName("Elevate with velocity");
  }

  public Command manualElevationCommand(CommandXboxController controller) {
    return run(
      () -> {
        double velocity = controller.getLeftY() * MANUAL_ELEVATION_VELOCITY;
        if(velocity >= MANUAL_ELEVATION_VELOCITY * MANUAL_ELEVATION_DEADBAND) {
          leftMotor.set(velocity);
          rightMotor.set(velocity);
        }
      }
    );
  }
}

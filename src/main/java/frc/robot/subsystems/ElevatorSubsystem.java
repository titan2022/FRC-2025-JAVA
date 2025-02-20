package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  // We have two Falcon 500s
  // TODO: Specify CAN IDs
  private static final TalonFX leftMotor = new TalonFX(0, "CANivore");
  private static final TalonFX rightMotor = new TalonFX(0, "CANivore");

  // We have a REV through-bore encoder
  // TODO: Specify DIO channels
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final Encoder encoder = new Encoder(0, 0);

  private static final ProfiledPIDController pid = new ProfiledPIDController(
    1.0, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      8, // max velocity in volts
      8 // max velocity in volts/second
    )
  );

  // TODO: Figure out good values for these constants
  // The unit is rotations of the encoder/elevator axle
  private static double POSITION_DEADBAND = 0.05;

  private static double MANUAL_ELEVATION_MAX_VOLTAGE = 0.5;
  private static double MANUAL_ELEVATION_DEADBAND = 0.15;

  private double currentVelocity;

  public ElevatorSubsystem() {
    // Brake the motors while not elevating
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    // TODO: Use a non-deprecated method
    // Invert the right motor
    rightMotor.setInverted(true);

    // Set default command to turn off the left and right motors, and then idle
    setDefaultCommand(
      runOnce(
        () -> {
          stopElevating();
        }
      )
      .andThen(run(() -> {}))
      .withName("Idle")
    );

    resetMeasurement();
  }

  public void resetMeasurement() {
    encoder.reset();
  }

  public double getMeasurement() {
    return encoder.getDistance();
  }

  public enum ElevationTarget {
    // TODO: Get the real values of these
    // Unit: rotations of the encoder/elevator axle
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

  public void elevateAtVoltage(double velocity) {
    currentVelocity = velocity;
    leftMotor.setVoltage(velocity);
    rightMotor.setVoltage(velocity);
  }

  public void stopElevating() {
    currentVelocity = 0;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private class ElevateCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double target;
    private double latestMeasurement;

    public ElevateCommand(ElevatorSubsystem elevator, double target) {
      this.elevator = elevator;
      this.target = target;
      addRequirements(elevator);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getMeasurement();
      elevateAtVoltage(pid.calculate(target, latestMeasurement));
    }

    @Override
    public boolean isFinished() {
      return Math.abs(latestMeasurement - target) < POSITION_DEADBAND;
    }
  }

  public Command elevateCommand(double target) {
    return new ElevateCommand(this, target);
  }

  public Command elevateCommand(ElevationTarget target) {
    return elevateCommand(target.getValue());
  }

  public Command applyVelocityCommand(double velocity) {
    return runOnce(
      () -> {
        elevateAtVoltage(velocity);
      }
    ).withName("Elevate with velocity");
  }

  public Command manualElevationCommand(CommandXboxController controller) {
    return run(
      () -> {
        double velocity = controller.getLeftY() * MANUAL_ELEVATION_MAX_VOLTAGE;
        if(velocity >= MANUAL_ELEVATION_MAX_VOLTAGE * MANUAL_ELEVATION_DEADBAND) {
          elevateAtVoltage(velocity);
        } else {
          stopElevating();
        }
      }
    );
  }
}

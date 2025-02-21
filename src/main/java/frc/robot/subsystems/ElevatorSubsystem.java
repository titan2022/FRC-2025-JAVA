package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  // We have two Falcon 500s
  // TODO: Specify CAN IDs
  private static final TalonFX leftMotor = new TalonFX(40, "rio");
  private static final TalonFX rightMotor = new TalonFX(41, "rio");

  // We have a REV through-bore encoder
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final Encoder encoder = new Encoder(0, 1);

  private static final boolean HAS_ENCODER = true;

  private static final ProfiledPIDController pid = new ProfiledPIDController(
    6.0, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      8, // max velocity in volts
      8 // max velocity in volts/second
    )
  );

  private double initialLeft = 0.0;
  private double initialRight = 0.0;

  private StatusSignal<Angle> leftPositionStatusSignal;
  private StatusSignal<Angle> rightPositionStatusSignal;

  // TODO: Figure out good values for these constants
  // The unit is rotations of the encoder/elevator axle
  private static double POSITION_DEADBAND = 0.05;

  private static double ELEVATION_GEAR_RATIO = 6.4;

  private static double MANUAL_UPWARDS_ELEVATION_MAX_VOLTAGE = 6.0;
  private static double MANUAL_DOWNWARDS_ELEVATION_MAX_VOLTAGE = 3.0;
  private static double MANUAL_ELEVATION_DEADBAND = 0.15;

  private double currentVelocity = 0;
  private double target = 0;
  private boolean isManuallyElevating = false;
  private int epoch = 0; // Increments every time we switch between manual and PID

  public ElevatorSubsystem() {
    // Brake the motors while not elevating
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    // TODO: Use a non-deprecated method
    // Invert the right motor
    rightMotor.setInverted(true);

    // Set default command to turn off the left and right motors, and then idle
    // During normal operation the ManualElevationCommand should be running
    setDefaultCommand(
      runOnce(
        () -> {
          stopElevating();
        }
      )
      .andThen(run(() -> {}))
      .withName("Idle")
    );

    if(!HAS_ENCODER) {
      leftPositionStatusSignal = leftMotor.getRotorPosition();
      rightPositionStatusSignal = rightMotor.getRotorPosition();
    }

    resetMeasurement();
  }

  public void resetMeasurement() {
    if(HAS_ENCODER) {
      encoder.reset();
    } else {
      leftPositionStatusSignal.refresh();
      initialLeft = leftPositionStatusSignal.getValueAsDouble();
      rightPositionStatusSignal.refresh();
      initialRight = rightPositionStatusSignal.getValueAsDouble();
    }
  }

  public double getMeasurement() {
    if(HAS_ENCODER) {
      return encoder.getDistance();
    } else {
      double left = leftPositionStatusSignal.getValueAsDouble();
      double right = rightPositionStatusSignal.getValueAsDouble();
      double leftMeasurement = left - initialLeft;
      double rightMeasurement = right - initialRight;
      return ((leftMeasurement + rightMeasurement)/2)/ELEVATION_GEAR_RATIO;
    }
  }

  public enum ElevationTarget {
    // https://www.desmos.com/calculator/ocl2iqiu7n
    // Unit: rotations of the encoder/elevator axle
    CoralIntake(0),
    L1(1.2633321268),
    L2(2.26316329414),
    L3(3.67201630267),
    AlgaeL2(1.8995883242),
    AlgaeL3(3.35388820397)
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
    private final int currentEpoch;
    private double latestMeasurement;

    public ElevateCommand(ElevatorSubsystem elevator, double target) {
      this.elevator = elevator;
      this.target = target;
      elevator.epoch++;
      this.currentEpoch = elevator.epoch;

      elevator.target = target;
      // We don't actually use the elevator
      // addRequirements(elevator);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getMeasurement();
      elevateAtVoltage(pid.calculate(target, latestMeasurement));
    }

    @Override
    public boolean isFinished() {
      return (
        Math.abs(latestMeasurement - target) < POSITION_DEADBAND
      ) || (
        elevator.epoch != currentEpoch
      );
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

  private class ManualElevationCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final CommandXboxController controller;

    private double latestMeasurement;

    public ManualElevationCommand(ElevatorSubsystem elevator, CommandXboxController controller) {
      this.elevator = elevator;
      this.controller = controller;
      addRequirements(elevator);
    }

    @Override // every 20ms
    public void execute() {
      double input = controller.getLeftY();
      if(input >= MANUAL_ELEVATION_DEADBAND) {
        if(!isManuallyElevating) {
          isManuallyElevating = true;
          elevator.epoch++;
        }
        elevateAtVoltage(input * MANUAL_UPWARDS_ELEVATION_MAX_VOLTAGE);
      } else if(input <= -MANUAL_ELEVATION_DEADBAND) {
        if(!isManuallyElevating) {
          isManuallyElevating = true;
          elevator.epoch++;
        }
        elevateAtVoltage(input * MANUAL_DOWNWARDS_ELEVATION_MAX_VOLTAGE);
      } else {
        if(isManuallyElevating) {
          isManuallyElevating = false;
          elevator.target = getMeasurement();
          elevator.epoch++;
        }
        latestMeasurement = getMeasurement();
        elevateAtVoltage(pid.calculate(target, latestMeasurement));
      }
    }
  }

  public Command manualElevationCommand(CommandXboxController controller) {
    return new ManualElevationCommand(null, controller);
  }
}

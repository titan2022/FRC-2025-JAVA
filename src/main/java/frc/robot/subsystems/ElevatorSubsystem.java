package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  private static final int MAX_HEIGH_ENCODER_VALUE = 7495;

  // We have two Falcon 500s
  // TODO: Specify CAN IDs
  private static final TalonFX leftMotor = new TalonFX(40, "rio");
  private static final TalonFX rightMotor = new TalonFX(41, "rio");

  // We have a REV through-bore encoder
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final Encoder encoder = new Encoder(0, 1);

  private static final boolean HAS_ENCODER = true;

  private static final ProfiledPIDController pid = new ProfiledPIDController(
    10, // kP
    0.0, // kI
    0.45, // kD
    new TrapezoidProfile.Constraints(
      4, // max velocity in volts
      2 // max velocity in volts/second
    )
  );

  private double initialLeft = 0.0;
  private double initialRight = 0.0;

  private StatusSignal<Angle> leftPositionStatusSignal;
  private StatusSignal<Angle> rightPositionStatusSignal;

  // TODO: Figure out good values for these constants
  // The unit is rotations of the encoder/elevator axle
  private static double POSITION_DEADBAND = 0.025;

  private static double ELEVATION_GEAR_RATIO = 6.4;

  private double currentVelocity = 0;
  private double target = 0;
  private boolean isManuallyElevating = true;
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

  private double getRevMeasurement() {
    return -encoder.getDistance();
  }

  private double getFloatHeight() {
    return getRevMeasurement() / MAX_HEIGH_ENCODER_VALUE;
  }

  public double getMeasurement() {
    if(HAS_ENCODER) {
      return getRevMeasurement();
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
    // L1(1.2633321268),
    // L2(2.26316329414),
    // L3(3.67201630267),
    L1(0.5),
    L2(0.75),
    L3(1.0),
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
    SmartDashboard.putNumber("elev V", velocity);
  }

  public void stopElevating() {
    currentVelocity = 0;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private class ElevateCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int currentEpoch;
    private double latestMeasurement;

    public ElevateCommand(ElevatorSubsystem elevator, double target) {
      target = Math.max(Math.min(target, 1.0), 0.0);

      this.elevator = elevator;
      elevator.epoch++;
      this.currentEpoch = elevator.epoch;

      elevator.target = target;
      // We don't actually use the elevator
      addRequirements(elevator);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getFloatHeight();
      double result = pid.calculate(target, latestMeasurement);
      SmartDashboard.putNumber("pre result", result);

      // Quick fix
      result = Math.max(Math.min(result, 2.0), -2.0);

      SmartDashboard.putNumber("result", result);

      elevateAtVoltage(result);
    }

    @Override
    public void end(boolean isInterrupted) {
      elevateAtVoltage(0);
    }

    @Override
    public boolean isFinished() {
      return false;
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
      double input = -controller.getLeftY();
      target = target + input * 0.01;
      target = Math.max(Math.min(target, 1.0), 0.0);

      if(isManuallyElevating) {
        isManuallyElevating = false;
        elevator.target = getFloatHeight();
        elevator.epoch++;
      }

      latestMeasurement = getFloatHeight();
      double calculation = pid.calculate(target, latestMeasurement);
      // calculation = Math.max(Math.min(calculation, 3.0), -3.0);
      elevateAtVoltage(calculation);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("height", getFloatHeight());
    SmartDashboard.putNumber("target", target);
  }

  public Command manualElevationCommand(CommandXboxController controller) {
    return new ManualElevationCommand(this, controller);
  }

  public void resetTarget() {
    target = getFloatHeight();
  }
}

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

import java.lang.annotation.Target;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  private static final int MAX_HEIGHT_REV_ENCODER_VALUE = 7495;
  private static final int MAX_HEIGHT_FALCON_ENCODER_VALUE = 0; // TODO: find this value
  private static final boolean HAS_ENCODER = true;
  private static final double MAX_VOLTAGE = 3.0;
  private static final double JOYSTICK_DEADBAND = 0.12;

  // We have two Falcon 500s
  // TODO: Specify CAN IDs
  private static final TalonFX leftMotor = new TalonFX(40, "rio");
  private static final TalonFX rightMotor = new TalonFX(41, "rio");

  // We have a REV through-bore encoder
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final Encoder encoder = new Encoder(0, 1);

  private static final ProfiledPIDController pid = new ProfiledPIDController(
    5.0, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      3.0, // max velocity in volts
      2.0 // max velocity in volts/second
    )
  );

  private double initialLeft = 0.0;
  private double initialRight = 0.0;

  private StatusSignal<Angle> leftPositionStatusSignal;
  private StatusSignal<Angle> rightPositionStatusSignal;

  private static double ELEVATION_GEAR_RATIO = 6.4;

  private double currentVelocity = 0;
  private double target = 0;
  private boolean hasStarted = false;

  public ElevatorSubsystem() {
    // Brake the motors while not elevating
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    // TODO: Use a non-deprecated method
    // Invert the right motor
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    // Invert encoder
    encoder.setReverseDirection(true);

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
    return encoder.getDistance();
  }

  private double getFalconMeasurement() {
    double left = leftPositionStatusSignal.getValueAsDouble();
    double right = rightPositionStatusSignal.getValueAsDouble();
    double leftMeasurement = left - initialLeft;
    double rightMeasurement = right - initialRight;
    return ((leftMeasurement + rightMeasurement)/2)/ELEVATION_GEAR_RATIO;
  }

  /*
   * Gets encoder measurement, returns elevator height from 0.0 to 1.0
   */
  public double getMeasurement() {
    if(HAS_ENCODER) {
      return getRevMeasurement() / MAX_HEIGHT_REV_ENCODER_VALUE;
    } else {
      return getFalconMeasurement() / MAX_HEIGHT_FALCON_ENCODER_VALUE;
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
    SmartDashboard.putNumber("Elevator Voltage", velocity);
  }

  public void stopElevating() {
    currentVelocity = 0;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  // Quick fix
  public void resetTarget() {
    target = getMeasurement();
  }

  private class ElevateCommand extends Command {
    private final ElevatorSubsystem elevator;
    private double latestMeasurement;

    public ElevateCommand(ElevatorSubsystem elevator, double target) {
      target = Math.max(Math.min(target, 1.0), 0.0);
      this.elevator = elevator;
      elevator.target = target;
      addRequirements(elevator);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getMeasurement();
      double calculation = pid.calculate(target, latestMeasurement);
      calculation = Math.max(Math.min(calculation, MAX_VOLTAGE), -MAX_VOLTAGE);
      elevator.elevateAtVoltage(calculation);
    }

    @Override
    public void end(boolean isInterrupted) {
      elevator.elevateAtVoltage(0);
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
    private final CommandXboxController controller;

    private double latestMeasurement;

    public ManualElevationCommand(ElevatorSubsystem elevator, CommandXboxController controller) {
      this.controller = controller;
      addRequirements(elevator);
    }

    @Override
    public void initialize() {
      // Should fix the set point command slamming?
      hasStarted = false;
    }

    @Override // every 20ms
    public void execute() {
      double input = -controller.getLeftY();
      if (Math.abs(input) > JOYSTICK_DEADBAND) {
        target = target + input * 0.02;
      }
      target = Math.max(Math.min(target, 1.0), 0.0);
      
      if(!hasStarted) {
        hasStarted = true;
        target = getMeasurement();
      }

      latestMeasurement = getMeasurement();
      double calculation = pid.calculate(target, latestMeasurement);
      calculation = Math.max(Math.min(calculation, MAX_VOLTAGE), -MAX_VOLTAGE);
      elevateAtVoltage(-calculation);
    }
  }

  public Command manualElevationCommand(CommandXboxController controller) {
    return new ManualElevationCommand(this, controller);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", getMeasurement());
    SmartDashboard.putNumber("Elevator Target Height", target);
    SmartDashboard.putNumber("REV Encoder", getRevMeasurement());
  }
}

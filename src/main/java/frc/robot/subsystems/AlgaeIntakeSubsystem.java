package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevationTarget;
import frc.robot.utility.Constants.Unit;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.path.RotationTarget;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private static final double MIN_ANGLE = 0; // 
  private static final double MAX_ANGLE = 65; //
  private static final double REV_OFFSET = 236.7578759189469-65; // Offset for REV absolute encoder
  private static final boolean USING_MOTION_MAGIC = false; // Uses `ProfiledPIDController` with REV absolute encoder if `false`
  private static final double MAX_VOLTAGE = 1.0;
  private static final double JOYSTICK_DEADBAND = 0.12;

  private static final TalonFX pivotMotor = new TalonFX(32, "rio");
  private static final TalonFX intakeRollersMotor = new TalonFX(21, "rio");

  // We have a REV through-bore encoder
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final DutyCycleEncoder encoder = new DutyCycleEncoder(0, 360, REV_OFFSET + MIN_ANGLE);

    private static final ProfiledPIDController pid = new ProfiledPIDController(
    0.0, // kP
    0.0, // kI
    0.0, // kD
    new TrapezoidProfile.Constraints(
      3.0, // max velocity in volts
      2.0 // max velocity in volts/second
    )
  );
  private static final ArmFeedforward feedforward = new ArmFeedforward(
    0.0,
    0.0, 
    0.0, 
    0.0
  );

  private double lastSpeed = 0;
  private double lastTime = 0;

  private double target = MAX_ANGLE;
  public AlgaeIntakeSubsystem() {
    SmartDashboard.putNumber("Pivot Target", target);
      SmartDashboard.putNumber("Encoder Measurement", getRevMeasurement());
  }

  public double getRevMeasurement() {
    if (!encoder.isConnected()) {
      return -1;
    }
    return encoder.get();
  }

  public void rotateAtVoltage(double velocity) {
    pivotMotor.setVoltage(0);
  }

  public void stopRotating() {
    goToRotation(getRevMeasurement());
  }

  public void resetTarget() {
    target = MAX_ANGLE;
  }
  public void goToRotation(double goalRotation) {
    double pidVal = pid.calculate(getRevMeasurement(), goalRotation);
    double acceleration = (pid.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    double voltage =  Math.max(Math.min(pidVal + feedforward.calculate(pid.getSetpoint().velocity, acceleration), 1), -1);
    pivotMotor.setVoltage(voltage);
    lastSpeed = pid.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  public enum AngleTarget {
    // Unit: Degrees
    Intake(MIN_ANGLE),
    Score(70), 
    Stow(MAX_ANGLE), 
    ;

    private double targetValue;
    private AngleTarget(double targetValue) {
      this.targetValue = targetValue;
    }
    public double getValue() {
      return targetValue;
    }
  }

  public Command rotateCommand(double target) {
    return new RotateCommand(this, target);
  }

  public Command rotateCommand(AngleTarget target) {
    return rotateCommand(target.getValue());
  }


  private class RotateCommand extends Command {
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
    private double latestMeasurement;

    public RotateCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem, double AngleTarget) {
      target = Math.max(Math.min(target, 1.0), 0.0);
      this.algaeIntakeSubsystem = algaeIntakeSubsystem;
      algaeIntakeSubsystem.target = AngleTarget;
      addRequirements(algaeIntakeSubsystem);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getRevMeasurement();
      algaeIntakeSubsystem.goToRotation(target);
      SmartDashboard.putNumber("Pivot Target", target);
      SmartDashboard.putNumber("Encoder Measurement", latestMeasurement);

    }

    @Override
    public void end(boolean isInterrupted) {
      algaeIntakeSubsystem.rotateAtVoltage(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
 

}

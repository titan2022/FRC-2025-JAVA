package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.annotation.Target;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Little summary of what we need to do for GroundCoralAlgaeIntakeSubsystem, according to my interpretation of Neel + my own efforts:

// We need to change the execute() method for the Intake/Score methods. Should probably have an Xbox button.
// I just put down a dummy value for that but that needs to change
// We need to find all the constants for the Coral values
// (done) We need intake coral and score coral commands
// (done) Becasue coral spins differently than algae the voltages
// should be opposite of algae values negative voltage (so the coral should be positive) 
// (not done) I can't find where the resetTarget() was used in the code
// So idk how best to add the coralResetTarget and algaeIntakeTarget into the rest of code
// (done) keep bool for coral intake status

public class GroundCoralAlgaeIntakeSubsystem extends SubsystemBase {

  // We need to have four angles for the dual algae/coral scorer
  // Coral pickup angle, Coral scoring angle, algae pickup angle, algae scoring
  // angle

  private static final double MAX_ANGLE = 60;
  private static final double STOW_ANGLE = 50;

  private static final double ALGAE_INTAKE_ANGLE = 35;
  private static final double ALGAE_SCORE_VALUE = 45;
  private static final double ALGAE_HOLD_VALUE = 40;

  private static final double CORAL_INTAKE_ANGLE = -45; 
  private static final double CORAL_SCORE_VALUE = 55; 
  private static final double CORAL_HOLD_VALUE = 40; 

  // TODO: calibrate this
  private static final double REV_OFFSET = -75; // Offset for REV absolute encoder

  private static final double MAX_VOLTAGE = 4.0;

  public static final double ALGAE_INTAKE_SPEED = 9;
  public static final double ALGAE_OUTTAKE_SPEED = 12;
  public static final double HOLD_ALGAE_INTAKE_VOLTAGE = 0.20;

  public static final double ALGAE_INTAKE_HAS_GP_CURRENT = 18;

  public static final double CORAL_INTAKE_SPEED = 4; 
  public static final double CORAL_OUTTAKE_SPEED = 8;
  public static final double HOLD_CORAL_INTAKE_VOLTAGE = 0.25; 

  public static final double CORAL_INTAKE_HAS_GP_CURRENT = 5; 

  public static final Time DEBOUNCE_TIME = Seconds.of(0.04);

  public static final double ANGLE_DEADBAND = 2;

  private static final TalonFX pivotMotor = new TalonFX(32, "rio");
  private static final TalonFX intakeRollersMotor = new TalonFX(55, "rio");
  // We have a REV through-bore encoder
  // Programming manual:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final DutyCycleEncoder encoder = new DutyCycleEncoder(0, 360, REV_OFFSET);

  private static final ProfiledPIDController pid = new ProfiledPIDController(
      0.07, // kP
      0.000, // kI
      0.000, // kD
      new TrapezoidProfile.Constraints(
          5000.0,
          5000.0));
  private static final ArmFeedforward feedforward = new ArmFeedforward(
      0.05,
      0.07,
      0.800,
      0.000);

  private double target = MAX_ANGLE;

  private final Debouncer debouncer;

  public GroundCoralAlgaeIntakeSubsystem() {
    encoder.setInverted(false);
    debouncer = new Debouncer(DEBOUNCE_TIME.in(Seconds), DebounceType.kRising);
  }

  public double getRevMeasurement() {
    if (!encoder.isConnected()) {
      return -1;
    }
    double measurement = encoder.get();
    // Prevent Looping around when encoder goes below 0
    if (measurement > 180)
      measurement -= 360;
    return measurement;
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
    SmartDashboard.putNumber("arm diff", goalRotation - getRevMeasurement());
    pid.setGoal(goalRotation);
    double pidVal = pid.calculate(getRevMeasurement());
    double velocity = pid.getSetpoint().velocity;
    double feedforwardval = feedforward.calculate(pid.getSetpoint().position, velocity);
    double voltage = Math.max(Math.min(pidVal + feedforwardval, MAX_VOLTAGE), -MAX_VOLTAGE);
    pivotMotor.setVoltage(-voltage);
    // SmartDashboard.putNumber("Voltage", voltage);
  }

  public enum AngleTarget {
    // Unit: Degrees
    AlgaeIntake(ALGAE_INTAKE_ANGLE),
    AlgaeScore(ALGAE_SCORE_VALUE),
    AlgaeHold(ALGAE_HOLD_VALUE),

    CoralIntake(CORAL_INTAKE_ANGLE),
    CoralScore(CORAL_SCORE_VALUE),
    CoralHold(CORAL_HOLD_VALUE),
    Stow(STOW_ANGLE);

    private double targetValue;

    private AngleTarget(double targetValue) {
      this.targetValue = targetValue;
    }

    public double getValue() {
      return targetValue;
    }
  }

  public void setGroundCoralAlgaeIntakeMotor(double speed) {
    intakeRollersMotor.set(speed);
  }

  public boolean hasAlgae() {
    boolean current = intakeRollersMotor.getStatorCurrent().getValue().in(Amp) > ALGAE_INTAKE_HAS_GP_CURRENT;
    return debouncer.calculate(current);
  }

  public boolean hasCoral() {
    boolean current = intakeRollersMotor.getStatorCurrent().getValue().in(Amp) > CORAL_INTAKE_HAS_GP_CURRENT;
    return debouncer.calculate(current);
  }

  public boolean atTarget() {
    return Math.abs(getRevMeasurement() - target) < ANGLE_DEADBAND;
  }

  public boolean getHasAlgae() {
    return hasAlgae();
  }

  public boolean getHasCoral() {
    return hasCoral();
  }

  public Command intakeCoralCommand() {
    return new GroundCoralIntakeCommand(this);
  }

  public class GroundCoralIntakeCommand extends Command {
    private final GroundCoralAlgaeIntakeSubsystem intake;

    public GroundCoralIntakeCommand(GroundCoralAlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {  
      intakeRollersMotor.setVoltage(CORAL_INTAKE_SPEED);
      target = AngleTarget.CoralIntake.getValue();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
      return hasCoral();
    }

    @Override
    public void end(boolean isInterrupted) {
      intakeRollersMotor.setVoltage(HOLD_CORAL_INTAKE_VOLTAGE);
      target = AngleTarget.CoralHold.getValue();
    }
  }

  public Command scoreCoralCommand() {
    return new GroundCoralScoreCommand(this);
  }

  public class GroundCoralScoreCommand extends Command {
    private final GroundCoralAlgaeIntakeSubsystem intake;

    public GroundCoralScoreCommand(GroundCoralAlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      target = AngleTarget.CoralScore.getValue();
    }

    @Override
    public void execute() {
      if(atTarget()){
        intakeRollersMotor.setVoltage(-CORAL_OUTTAKE_SPEED);
      }
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean isInterrupted) {
      intakeRollersMotor.setVoltage(0);
      target = AngleTarget.Stow.getValue();
    }
  }

  public Command intakeAlgaeCommand() {
    return new GroundAlgaeIntakeCommand(this);
  }

  public class GroundAlgaeIntakeCommand extends Command {
    private final GroundCoralAlgaeIntakeSubsystem intake;

    public GroundAlgaeIntakeCommand(GroundCoralAlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intakeRollersMotor.setVoltage(-ALGAE_INTAKE_SPEED);
      target = AngleTarget.AlgaeIntake.getValue();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
      return hasAlgae();
    }

    @Override
    public void end(boolean isInterrupted) {
      intakeRollersMotor.setVoltage(-HOLD_ALGAE_INTAKE_VOLTAGE);
      target = AngleTarget.AlgaeHold.getValue();
    }
  }

  public Command scoreAlgaeCommand() {
    return new GroundAlgaeScoreCommand(this);
  }

  public class GroundAlgaeScoreCommand extends Command {
    private final GroundCoralAlgaeIntakeSubsystem intake;

    public GroundAlgaeScoreCommand(GroundCoralAlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      target = AngleTarget.AlgaeScore.getValue();
    }

    @Override
    public void execute() {
      if(atTarget())
         intakeRollersMotor.setVoltage(ALGAE_OUTTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean isInterrupted) {
      intakeRollersMotor.setVoltage(0);
      target = AngleTarget.Stow.getValue();
    }
  }

  @Override
  public void periodic() {
     goToRotation(target);
     SmartDashboard.putNumber("Pivot Target", target);
     SmartDashboard.putNumber("Encoder Measurement", getRevMeasurement());
    // SmartDashboard.putBoolean("has Algae", hasFinishedIntakingAlgae());
    // SmartDashboard.putBoolean("has Coral", hasFinishedIntakingCoral());
    // SmartDashboard.putString("intake Velocity",
    // intakeRollersMotor.getVelocity().getValue().toString());
    // SmartDashboard.putString("intake Current",
    // intakeRollersMotor.getStatorCurrent().getValue().toString());

  }
}

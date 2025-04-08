package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Little summary of what we need to do for GroundCoralAlgaeIntakeSubsystem, according to my interpretation of Neel:

// We need intake coral and score coral commands
// Becasue coral spins differently than algae the voltages 
// should be opposite of algae values negative voltage (so the coral should be positive) 
// add (algae/coral)ResetTarget into the rest of code

// keep bool for coral intake status



public class GroundCoralAlgaeIntakeSubsystem extends SubsystemBase {

  // We need to have four angles for the dual algae/coral scorer
  // Coral pickup angle, Coral scoring angle, algae pickup angle, algae scoring angle
  
  private static final double ALGAE_MIN_ANGLE = 90;
  private static final double CORAL_MIN_ANGLE = 0; // !!! Need this set
  private static final double CORAL_MAX_ANGLE = 0; // !!! Need this set
  private static final double ALGAE_MAX_ANGLE = 35;

  private static final double ALGAE_SCORE_VALUE = 65;
  private static final double ALGAE_HOLD_VALUE = 60;

  private static final double CORAL_SCORE_VALUE = 0; // !!! Need this set
  private static final double CORAL_HOLD_VALUE = 0; // !!! Need this set
  
  private static final double REV_OFFSET = -160; // Offset for REV absolute encoder
  private static final boolean USING_MOTION_MAGIC = false; // Uses `ProfiledPIDController` with REV absolute encoder if
                                                           // `false`
  private static final double MAX_VOLTAGE = 4.0;
  private static final double JOYSTICK_DEADBAND = 0.12;

  public static final double ALGAE_INTAKE_SPEED = 9;
  public static final double ALGAE_OUTTAKE_SPEED = -12;
  public static final double HOLD_ALGAE_INTAKE_VOLTAGE = 0.20;
  public static final AngularVelocity ALGAE_INTAKE_HAS_GP_VELOCITY = RotationsPerSecond.of(-4500 / 60);
  public static final Current ALGAE_INTAKE_HAS_GP_CURRENT = Amps.of(4.5);

  public static final double CORAL_INTAKE_SPEED = 0; // !!! Need this set
  public static final double CORAL_OUTTAKE_SPEED = 0; // !!! Need this set
  public static final double HOLD_CORAL_INTAKE_VOLTAGE = 0; // !!! Need this set
  public static final AngularVelocity CORAL_INTAKE_HAS_GP_VELOCITY = RotationsPerSecond.of(0); // !!! Need this set
  public static final Current CORAL_INTAKE_HAS_GP_CURRENT = Amps.of(0); // !!! Need this set



  private static final TalonFX pivotMotor = new TalonFX(32, "rio");
  private static final TalonFX intakeRollersMotor = new TalonFX(55, "rio");
  private boolean hasAlgae = false;
  private boolean hasCoral = false;

  // We have a REV through-bore encoder
  // Programming manual:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final DutyCycleEncoder encoder = new DutyCycleEncoder(0, 360, REV_OFFSET);

  private static final ProfiledPIDController pid = new ProfiledPIDController(
      0.15, // kP
      0.000, // kI
      0.005, // kD
      new TrapezoidProfile.Constraints(
          5000.0,
          5000.0));
  private static final ArmFeedforward feedforward = new ArmFeedforward(
      0.05,
      0.07,
      0.800,
      0.000);

  private double lastSpeed = 0;
  private double lastTime = 0;

  private double target = ALGAE_MIN_ANGLE; 
  // We might want to change this to CORAL_MIN_ANGLE 
  // depending on which we want to score first

  public GroundCoralAlgaeIntakeSubsystem() {
    encoder.setInverted(false);
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
  

  public void algaeResetTarget() {
    target = ALGAE_MAX_ANGLE;
  }
  public void coralResetTarget() {
    target = CORAL_MAX_ANGLE;
  }

  public void goToRotation(double goalRotation) {
    pid.setGoal(goalRotation);
    double pidVal = pid.calculate(getRevMeasurement());
    double velocity = pid.getSetpoint().velocity;
    double feedforwardval = feedforward.calculate(pid.getSetpoint().position, velocity);
    double voltage = Math.max(Math.min(pidVal + feedforwardval, MAX_VOLTAGE), -MAX_VOLTAGE);
    pivotMotor.setVoltage(-voltage);
    lastSpeed = velocity;
    lastTime = Timer.getFPGATimestamp();
    // SmartDashboard.putNumber("Voltage", voltage);
  }


  public enum AngleTarget {
    // Unit: Degrees
    AlgaeIntake(ALGAE_MAX_ANGLE),
    AlgaeScore(ALGAE_SCORE_VALUE),
    AlgaeHold(ALGAE_HOLD_VALUE),
    AlgaeStow(ALGAE_MIN_ANGLE),

    CoralIntake(CORAL_MAX_ANGLE),
    CoralScore(CORAL_SCORE_VALUE),
    CoralHold(CORAL_HOLD_VALUE), 
    CoralStow(CORAL_MIN_ANGLE);

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
    private final GroundCoralAlgaeIntakeSubsystem groundCoralAlgaeIntakeSubsystem;
    private double latestMeasurement;
    private double target;

    public RotateCommand(GroundCoralAlgaeIntakeSubsystem groundCoralAlgaeIntakeSubsystem, double AngleTarget) {
      target = Math.max(Math.min(target, 1.0), 0.0);
      this.groundCoralAlgaeIntakeSubsystem = groundCoralAlgaeIntakeSubsystem;
      this.target = AngleTarget;
      addRequirements(groundCoralAlgaeIntakeSubsystem);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getRevMeasurement();
      groundCoralAlgaeIntakeSubsystem.target = target; // We may want to change this
    }

    @Override
    public void end(boolean isInterrupted) {
      groundCoralAlgaeIntakeSubsystem.rotateAtVoltage(0.5);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  public void setGroundCoralAlgaeIntakeMotor(double speed) {
    intakeRollersMotor.set(speed);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = intakeRollersMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = intakeRollersMotor.getVelocity().getValue();
    double intakeAcceleration = intakeRollersMotor.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = ALGAE_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = ALGAE_INTAKE_HAS_GP_VELOCITY;

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.gte(intakeHasGamePieceVelocity))) {
      return true;
    } else {
      return false;
    }
  }

  public boolean hasCoral() {
    //mostly same from hasAlgae()
    Current intakeCurrent = intakeRollersMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = intakeRollersMotor.getVelocity().getValue();
    double intakeAcceleration = intakeRollersMotor.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = CORAL_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = CORAL_INTAKE_HAS_GP_VELOCITY;

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.gte(intakeHasGamePieceVelocity))) {
      return true;
    } else {
      return false;
    }
  }

  public void startCoralIntaking() {
    intakeRollersMotor.setVoltage(-ALGAE_INTAKE_SPEED);
    target = AngleTarget.CoralIntake.getValue();
    hasCoral = true;
  }
  
  public void startAlgaeIntaking() {
    intakeRollersMotor.setVoltage(CORAL_INTAKE_SPEED);
    // ^^^ I believe the CORAL_INTAKE_SPEED should have no negative because it
    // needs to be the opposite of the ALGAE_INTAKE_SPEED but correct if wrong
    target = AngleTarget.AlgaeIntake.getValue();
    hasAlgae = true;
  }

  public void startAlgaeScoring() {
    intakeRollersMotor.setVoltage(-ALGAE_OUTTAKE_SPEED);
    target = AngleTarget.AlgaeScore.getValue();
  }

  public void startCoralScoring() {
    intakeRollersMotor.setVoltage(CORAL_OUTTAKE_SPEED);
    // Positive Coral Outtake speed because of same rationale above
    target = AngleTarget.CoralScore.getValue();
  }

  public void stopAlgaeScoring() {
    intakeRollersMotor.setVoltage(0);
    target = AngleTarget.AlgaeStow.getValue();
    hasAlgae = false;
  }

  public void stopCoralScoring() {
    intakeRollersMotor.setVoltage(0);
    target = AngleTarget.CoralStow.getValue();
    hasCoral = false;
  }

  public void stopAlgaeIntaking() {
    intakeRollersMotor.setVoltage(-HOLD_ALGAE_INTAKE_VOLTAGE);
    target = AngleTarget.AlgaeHold.getValue();

  }

  public void stopCoralIntaking() {
    intakeRollersMotor.setVoltage(HOLD_CORAL_INTAKE_VOLTAGE);
    // Positive Coral Outtake speed because of same rationale above
    target = AngleTarget.CoralHold.getValue();

  }

  public boolean getHasAlgae() {
    return hasAlgae;
  }
  
  public boolean getHasCoral() {
    return hasCoral;
  }

  public Command intakeCommand() {
    return new AlgaeIntakeCommand(this);
  }

  public class AlgaeIntakeCommand extends Command {
    private final GroundCoralAlgaeIntakeSubsystem intake;

    public AlgaeIntakeCommand(GroundCoralAlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      if (!hasAlgae())
        intake.startIntaking();
      else {
        intake.stopIntaking();
      }

    }

    @Override
    public boolean isFinished() {
      // return hasAlgae();
      return false;
    }

    @Override
    public void end(boolean isInterrupted) {
      intake.stopIntaking();
    }
  }

  public Command scoreCommand() {
    return new GroundCoralAlgaeScoreCommand(this);
  }

  public class GroundCoralAlgaeScoreCommand extends Command {
    private final GroundCoralAlgaeIntakeSubsystem intake;

    public GroundCoralAlgaeScoreCommand(GroundCoralAlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      intake.startScoring();

    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean isInterrupted) {
      intake.stopScoring();
    }
  }

  @Override
  public void periodic() {
    goToRotation(target);
    // SmartDashboard.putNumber("Pivot Target", target);
    // SmartDashboard.putNumber("Encoder Measurement", getRevMeasurement());
    // SmartDashboard.putBoolean("has Algae", hasAlgae());
    // SmartDashboard.putString("intake Velocity",
    // intakeRollersMotor.getVelocity().getValue().toString());
    // SmartDashboard.putString("intake Current",
    // intakeRollersMotor.getStatorCurrent().getValue().toString());

  }
}

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevationTarget;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.path.RotationTarget;


public class AlgaeIntakeSubsystem extends SubsystemBase {
  private static final double MIN_ANGLE = 70;    
  private static final double MAX_ANGLE = 20; 
  private static final double REV_OFFSET = -140; // Offset for REV absolute encoder 
  private static final boolean USING_MOTION_MAGIC = false; // Uses `ProfiledPIDController` with REV absolute encoder if `false`
  private static final double MAX_VOLTAGE = 4.0;
  private static final double JOYSTICK_DEADBAND = 0.12;

  public static final double ALGAE_INTAKE_SPEED = 9;
  public static final double ALGAE_OUTTAKE_SPEED = -12;
  public static final double HOLD_ALGAE_INTAKE_VOLTAGE = 0.20;
  public static final AngularVelocity ALGAE_INTAKE_HAS_GP_VELOCITY = RotationsPerSecond.of(-5000 / 60);
  public static final Current ALGAE_INTAKE_HAS_GP_CURRENT = Amps.of(4.0);
  
  private static final TalonFX pivotMotor = new TalonFX(32, "rio");
  private static final TalonFX intakeRollersMotor = new TalonFX(21, "rio");

  // We have a REV through-bore encoder
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final DutyCycleEncoder encoder = new DutyCycleEncoder(0, 360, REV_OFFSET);

    private static final ProfiledPIDController pid = new ProfiledPIDController(
    0.1, // kP
    0.01, // kI
    0.001, // kD
    new TrapezoidProfile.Constraints(
      5000.0,
      5000.0
    )
  );
  private static final ArmFeedforward feedforward = new ArmFeedforward(
    0.08 ,
    0.16, 
    0.100, 
    0.000
  );

  private double lastSpeed = 0;
  private double lastTime = 0;

  private double target = MIN_ANGLE;
  public AlgaeIntakeSubsystem() {
    encoder.setInverted(false);
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
    pid.setGoal(goalRotation);
    double pidVal = pid.calculate(getRevMeasurement());
    double velocity = pid.getSetpoint().velocity;
    double feedforwardval = feedforward.calculate(pid.getSetpoint().position, velocity);
    double voltage =  Math.max(Math.min(-pidVal - feedforwardval, MAX_VOLTAGE), -MAX_VOLTAGE);
    pivotMotor.setVoltage(voltage);
    lastSpeed = velocity;
    lastTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Voltage", voltage);
  }

  public enum AngleTarget {
    // Unit: Degrees
    Intake(MAX_ANGLE),
    Score(45), 
    Hold(45), 
    Stow(MIN_ANGLE)
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
    private double target;

    public RotateCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem, double AngleTarget) {
      target = Math.max(Math.min(target, 1.0), 0.0);
      this.algaeIntakeSubsystem = algaeIntakeSubsystem;
      this.target = AngleTarget;
      addRequirements(algaeIntakeSubsystem);
    }

    @Override // every 20ms
    public void execute() {
      latestMeasurement = getRevMeasurement();
      algaeIntakeSubsystem.target=target;
    }

    @Override
    public void end(boolean isInterrupted) {
      algaeIntakeSubsystem.rotateAtVoltage(0.5);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
  public void setAlgaeIntakeMotor(double speed) {
    intakeRollersMotor.set(speed);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = intakeRollersMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = intakeRollersMotor.getVelocity().getValue();
    double intakeAcceleration = intakeRollersMotor.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = ALGAE_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = ALGAE_INTAKE_HAS_GP_VELOCITY;

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.gte(intakeHasGamePieceVelocity))
       ) {
      return true;
    } else {
      return false;
    }
  }

  public void startIntaking() {
    intakeRollersMotor.setVoltage(-ALGAE_INTAKE_SPEED);
    target = AngleTarget.Intake.getValue();
  }

  public void startScoring() {
    intakeRollersMotor.setVoltage(-ALGAE_OUTTAKE_SPEED);
    target = AngleTarget.Score.getValue();
  }

  public void stopScoring() {
    intakeRollersMotor.setVoltage(0);
    target = AngleTarget.Stow.getValue();
  }


  public void stopIntaking() {
    intakeRollersMotor.setVoltage(-HOLD_ALGAE_INTAKE_VOLTAGE);
    target = AngleTarget.Hold.getValue();

  }

  public Command intakeCommand() {
    return new AlgaeIntakeCommand(this);
  }

  public class AlgaeIntakeCommand extends Command {
    private final AlgaeIntakeSubsystem intake;
  
    public AlgaeIntakeCommand(AlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }
  
    @Override
    public void initialize() {
      
    }

    @Override
    public void execute(){
      if(!hasAlgae())
        intake.startIntaking();
      else{
        intake.stopIntaking();
      }

    }
  
    @Override
    public boolean isFinished() {
      return hasAlgae();
    }
    @Override
    public void end(boolean isInterrupted){
      intake.stopIntaking();
    }
  }

  public Command scoreCommand() {
    return new AlgaeScoreCommand(this);
  }

  public class AlgaeScoreCommand extends Command {
    private final AlgaeIntakeSubsystem intake;
  
    public AlgaeScoreCommand(AlgaeIntakeSubsystem intake) {
      this.intake = intake;
      addRequirements(intake);
    }
  
    @Override
    public void initialize() {
      
    }

    @Override
    public void execute(){
      intake.startScoring();

    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
    @Override
    public void end(boolean isInterrupted){
      intake.stopScoring();
    }
  }
  @Override
  public void periodic() {
    goToRotation(target);
    SmartDashboard.putNumber("Pivot Target", target);
    SmartDashboard.putNumber("Encoder Measurement", getRevMeasurement());
    SmartDashboard.putBoolean("has Algae", hasAlgae());
    SmartDashboard.putString("intake Velocity", intakeRollersMotor.getVelocity().getValue().toString());
    SmartDashboard.putString("intake Current", intakeRollersMotor.getStatorCurrent().getValue().toString());

  }
}

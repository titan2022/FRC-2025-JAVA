package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants.Unit;

import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private static final double MIN_ANGLE = 0 * Unit.DEG;
  private static final double MAX_ANGLE = 0 * Unit.DEG;
  private static final double REV_OFFSET = 0; // Offset for REV absolute encoder
  private static final boolean USING_MOTION_MAGIC = false; // Uses `ProfiledPIDController` with REV absolute encoder if `false`
  private static final double MAX_VOLTAGE = 1.0;
  private static final double JOYSTICK_DEADBAND = 0.12;

  private static final TalonFX pivotMotor = new TalonFX(32, "rio");
  private static final TalonFX intakeRollersMotor = new TalonFX(21, "rio");

  // We have a REV through-bore encoder
  // Programming manual: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#quadrature-encoders-the-encoder-class
  private static final DutyCycleEncoder encoder = new DutyCycleEncoder(0, 0, 0);

  public AlgaeIntakeSubsystem() {

  }

  public double getRevMeasurement() {
    if (!encoder.isConnected()) {
      return 0;
    }
    return encoder.get();
  }

  public enum AngleTarget {
    Bottom(0),
    Top(0);
    
    private double targetValue;
    private AngleTarget(double targetValue) {
      this.targetValue = targetValue;
    }
    public double getValue() {
      return targetValue;
    }
  }

  public void rotateAtVoltage(double velocity) {
    throw new UnsupportedOperationException();
  }

  public void stopRotating() {
    throw new UnsupportedOperationException();
  }

  public void resetTarget() {
    throw new UnsupportedOperationException();
  }

  
}

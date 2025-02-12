package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private static final TalonFX pivotMotor = new TalonFX(0, "CANivore");
  private static final TalonFX intakeMotor = new TalonFX(0, "CANivore");

  public AlgaeIntakeSubsystem() {

  }

  
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * An active intake that uses wheels to speed up the coral dropped in
 */
public class IntakeSubsystemCoral extends SubsystemBase {

    private CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    private MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    private FeedbackConfigs feedbackConfig = new FeedbackConfigs();

    private static final TalonFX wheelMotor = new TalonFX(3);
    private static TalonFXConfigurator talonFXConfigurator = wheelMotor.getConfigurator();

    public IntakeSubsystemCoral() {
        config();
    }

    public void config() {
        // the config values are those of last year
        // i do not know what the proper values here should be so please change this if
        // you do know.
        // Values:
        // SupplyCurrentLimitEnable True
        // SupplyCurrentLimit 30
        // SupplyCurrentLowerLimit 30
        // SupplyCurrentLowerTime 0
        limitConfig.SupplyCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLimit = 30;
        limitConfig.SupplyCurrentLowerLimit = 30;
        limitConfig.SupplyCurrentLowerTime = 0;

        /*
         * Having issues replicating this:
         * wheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
         * if anyone knows the new api for it feel free to replace
         */


        //this might be wrong, if issue try InvertedValue.CounterClockwise_Positive
        motorConfig.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.NeutralMode = NeutralModeValue.Coast;
        
        talonFXConfigurator.apply(limitConfig);
        talonFXConfigurator.apply(motorConfig);
    }
     /***  
   * Sets the speed of the wheels
   * @param speed In percentage from -1 to 1
   */
    public void setWheelSpeed(double speed) {
        wheelMotor.set(speed);
    }

    public double getWheelSpeed() {
        return wheelMotor.get();
    }

    public void toggle() {
        if (Math.abs(getWheelSpeed()) <= 0.01) {
          intake();
        } else {
          stop();
        }
      }
    
    public void intake() {
        setWheelSpeed(0.5);
    }
    public void stop() {
        setWheelSpeed(0);
    }
}

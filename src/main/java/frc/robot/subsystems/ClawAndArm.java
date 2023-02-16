package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;

public class ClawAndArm extends SubsystemBase {
    private static ClawAndArm instance;
    private AnalogPotentiometer pot;
    private Solenoid piston;
    private DigitalInput bottomHallEffect, topHallEffect;
    private TalonSRX rotatingMotor; 
    private TalonSRX extendingMotor; 
   
    public ClawAndArm() {
        pot = new AnalogPotentiometer(clawConstants.POTENTIOMETER_PORT, clawConstants.MAX_RESISTANCE, clawConstants.ANGLE_OFFSET); 
        piston = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.PISTON_PORT);

        bottomHallEffect = new DigitalInput(clawConstants.BOTTOMHALLEFFECT_PORT);
        topHallEffect = new DigitalInput(clawConstants.TOPHALLEFFECT_PORT);

        rotatingMotor = new WPI_TalonSRX(clawConstants.ROTATINGMOTOR_PORT);
        extendingMotor = new WPI_TalonSRX(clawConstants.EXTENDINGMOTOR_PORT);

        extendingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, clawConstants.ENCODER_PID_ID, clawConstants.ENCODER_TIMEOUT_MS); 
    }
   
    public static ClawAndArm getInstance() {
        if (instance == null) {
          instance = new ClawAndArm();
        }
        return instance;
    }

    public void setRotatingMotor(double speed) {
        rotatingMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setExtendingMotor(double speed) {
        extendingMotor.set(ControlMode.PercentOutput, speed);
    }

    public boolean getTopStatus() {
        return topHallEffect.get();
    }

    public boolean getBottomStatus () {
        return bottomHallEffect.get();
    }

    public double getAngle() {
        return pot.get() * clawConstants.VOLTAGE_TO_DEGREES_CONSTANT; 
    }

    public double getPosition() {
        return extendingMotor.getSelectedSensorPosition() * clawConstants.TICKS_TO_INCHES_CONSTANT;
    }

    public void extendPiston() {
        piston.set(true);
    }

    public void retractPiston() {
        piston.set(false);
    }


}
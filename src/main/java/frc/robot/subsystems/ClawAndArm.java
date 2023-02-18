package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.clawConstants;

public class ClawAndArm extends SubsystemBase {
    private static ClawAndArm instance;
    private AnalogPotentiometer pot;
    private Solenoid piston;
    private DigitalInput topHallEffect;
    private DigitalInput bottomHallEffect;
    private TalonFX rotationMotor; //subject to change
    private TalonFX extendingMotor; //subject to change
   
    public ClawAndArm() {
        pot = new AnalogPotentiometer(clawConstants.POTENTIOMETER_CANID, clawConstants.MAX_RESISTANCE, clawConstants.ANGLE_OFFSET); // double check values
        piston = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.PISTON_CANID);

        topHallEffect = new DigitalInput(Constants.clawConstants.TOP_PORT);
        bottomHallEffect = new DigitalInput(Constants.clawConstants.BOTTOM_PORT);
    }
   
    public static ClawAndArm getInstance() {
        if (instance == null) {
          instance = new ClawAndArm();
        }
        return instance;
    }

    public void setRotatingMotor(double speed) {
        // TODO: add potentiometer check to act as software limit
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setExtendingMotor(double speed) {
        if (getTopStatus() || getBottomStatus()) {
            extendingMotor.set(ControlMode.PercentOutput, 0);
            return;
        }
        extendingMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getVoltage() {
        return pot.get();
    }

    public double getAngle() {
        return getVoltage() * clawConstants.VOLTAGE_TO_ANGLE_CONSTANT; //to be calculated
    }

    public void closeClaw() {
        piston.set(true);
    }

    public void openClaw() {
        piston.set(false);
    } 

    public void toggleClaw(){
        piston.set(!piston.get());
    }

    public boolean getTopStatus() {
        return topHallEffect.get();
    }


    public boolean getBottomStatus() {
        return bottomHallEffect.get();
    }


}
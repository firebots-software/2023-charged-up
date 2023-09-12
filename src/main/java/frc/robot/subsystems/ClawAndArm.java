package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;

public class ClawAndArm extends SubsystemBase {
    private static ClawAndArm instance;
    private AnalogPotentiometer pot;
    private Solenoid piston;
    private TalonFX rotationMotor; //subject to change
    private TalonFX extendingMotor; //subject to change
   
    public ClawAndArm() {
        pot = new AnalogPotentiometer(clawConstants.POTENTIOMETER_CANID, clawConstants.MAX_RESISTANCE, clawConstants.ANGLE_OFFSET); // double check values
        piston = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.PISTON_CANID);
    }
   
    public static ClawAndArm getInstance() {
        if (instance == null) {
          instance = new ClawAndArm();
        }
        return instance;
    }
    /**
     * Sets speed for the arm rotation motor
     * @param speed speed for the rotation motor
     */
    public void setRotatingMotor(double speed) {
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }
   
    public void setExtendingMotor(double speed) {
        extendingMotor.set(ControlMode.PercentOutput, speed);
    }
    /**
     * gets the voltage of the potentiometer depending on the angle of the arm
     * @return voltage of potentiometer in that position
     */
    public double getVoltage() {
        return pot.get();
    }
    /**
     * angle is calculated from the voltage from the potentiometer
     * @return angle of the arm
     */
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


}
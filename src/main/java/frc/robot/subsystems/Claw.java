package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private AnalogPotentiometer pot;
    private Solenoid piston;
   
    public Claw() {
        pot = new AnalogPotentiometer(Constants.clawConstants.POTENTIOMETER_CANID, Constants.clawConstants.MAX_RESISTANCE, Constants.clawConstants.ANGLE_OFFSET); // double check values\
        piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.clawConstants.PISTON_CANID);
    }
   
    public double getVoltage() {
        return pot.get();
    }




    public double getAngle() {
        return getVoltage() * Constants.clawConstants.VOLTAGE_TO_ANGLE_CONSTANT;
    }




    public void extendPiston() {
        piston.set(true);
    }




    public void retractPiston() {
        piston.set(false);
    }


}
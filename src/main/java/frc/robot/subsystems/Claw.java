package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private AnalogPotentiometer pot;
    private Solenoid piston;
   
    public Claw() {
        pot = new AnalogPotentiometer(clawConstants.POTENTIOMETER_CANID, clawConstants.MAX_RESISTANCE, clawConstants.ANGLE_OFFSET); // double check values\
        piston = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.PISTON_CANID);
    }
   
    public static Claw getInstance() {
        if (instance == null) {
          instance = new Claw();
        }
        return instance;
    }

    public double getVoltage() {
        return pot.get();
    }

    public double getAngle() {
        return getVoltage() * clawConstants.VOLTAGE_TO_ANGLE_CONSTANT;
    }

    public void extendPiston() {
        piston.set(true);
    }

    public void retractPiston() {
        piston.set(false);
    }


}
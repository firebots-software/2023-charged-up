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

    public void setRotatingMotor(double speed) {
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setExtendingMotor(double speed) {
        extendingMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getDegrees() {
        return pot.get();
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
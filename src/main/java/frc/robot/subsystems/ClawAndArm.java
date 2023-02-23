package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;


public class ClawAndArm extends SubsystemBase {
    private static ClawAndArm instance;
    private AnalogPotentiometer pot;
    private Solenoid clawSolenoid;
    private Solenoid frictionBreakSolenoid;
    private DigitalInput bottomHallEffect, topHallEffect;
    private WPI_TalonFX rotatingMotor; 
    //private WPI_TalonSRX extendingMotor; 
   
    public ClawAndArm() {
        pot = new AnalogPotentiometer(clawConstants.POTENTIOMETER_PORT, clawConstants.RANGE_OF_MOTION, clawConstants.STARTING_POINT); 
        // clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.CLAW_SOLENOID_PORT);
        // frictionBreakSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.FRICTION_BREAK_PORT);

        // bottomHallEffect = new DigitalInput(clawConstants.BOTTOMHALLEFFECT_PORT);
        // topHallEffect = new DigitalInput(clawConstants.TOPHALLEFFECT_PORT);

        rotatingMotor = new WPI_TalonFX(clawConstants.ROTATINGMOTOR_PORT);
       // extendingMotor = new WPI_TalonSRX(clawConstants.EXTENDINGMOTOR_PORT);

       // extendingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, clawConstants.ENCODER_PID_ID, clawConstants.ENCODER_TIMEOUT_MS); 
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

    // public void setExtendingMotor(double speed) {
    //     extendingMotor.set(ControlMode.PercentOutput, speed);
    // }

    // public boolean getTopStatus() {
    //     return topHallEffect.get();
    // }

    // public boolean getBottomStatus () {
    //     return bottomHallEffect.get();
    // }

    public double getAngle() {
        return pot.get() * clawConstants.VOLTAGE_TO_DEGREES_CONSTANT; 
    }

    // public double getPosition() {
    //     double radians = extendingMotor.getSelectedSensorPosition() * clawConstants.TICKS_TO_RADIANS_CONSTANT;
    //     return 2*Math.PI*(0.375 + (0.04*radians)); //equation returns inches extended
    // }

    // public void closeClaw() {
    //     clawSolenoid.set(true);
    // }

    // public void openClaw() {
    //     clawSolenoid.set(false);
    // }

    // public void frictionBreakOn() {
    //     frictionBreakSolenoid.set(true);
    // }

    // public void frictionBreakOff() {
    //     frictionBreakSolenoid.set(false);
    // }

    public double getPot() {
        return pot.get();
    }

    @Override
    public void periodic() {
        System.out.println("hi");
        SmartDashboard.putNumber("potentiometer", pot.get());
    }


}
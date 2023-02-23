// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;

public class ClawAndArm2 extends SubsystemBase {
  private static ClawAndArm2 instance;
  private AnalogPotentiometer pot;
  private Solenoid clawSolenoid;
  private Solenoid frictionBreakSolenoid;
  private DigitalInput bottomHallEffect, topHallEffect;
  private TalonSRX rotatingMotor;
  private TalonSRX extendingMotor;

  /** Creates a new ClawAndArm2. */
  public ClawAndArm2() {
    pot = new AnalogPotentiometer(clawConstants.POTENTIOMETER_PORT, clawConstants.RANGE_OF_MOTION,
        clawConstants.STARTING_POINT);
    //clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.CLAW_SOLENOID_PORT);
    frictionBreakSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clawConstants.FRICTION_BREAK_PORT);

    //bottomHallEffect = new DigitalInput(clawConstants.BOTTOMHALLEFFECT_PORT);
    //topHallEffect = new DigitalInput(clawConstants.TOPHALLEFFECT_PORT);

    //rotatingMotor = new WPI_TalonSRX(clawConstants.ROTATINGMOTOR_PORT);
    //extendingMotor = new WPI_TalonSRX(clawConstants.EXTENDINGMOTOR_PORT);

    //extendingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, clawConstants.ENCODER_PID_ID,
    //   clawConstants.ENCODER_TIMEOUT_MS);
  }

  public static ClawAndArm2 getInstance() {
    if (instance == null) {
      instance = new ClawAndArm2();
    }
    return instance;
  }

  public double getPot() {
    return pot.get();
  }

      public void frictionBreakOn() {
        frictionBreakSolenoid.set(true);
    }

    public void frictionBreakOff() {
        frictionBreakSolenoid.set(false);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("potentiometer", getPot());
  }
}

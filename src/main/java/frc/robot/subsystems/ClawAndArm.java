// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.ejml.dense.row.decomposition.eig.watched.WatchedDoubleStepQREigenvector_FDRM;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.clawConstants;

public class ClawAndArm extends SubsystemBase {
  private static ClawAndArm instance;
  // private AnalogPotentiometer pot;
  private Solenoid frictionBreakSolenoid;
  private WPI_TalonFX rotatingMotor;

  /** Creates a new ClawAndArm2. */
  public ClawAndArm() {
    frictionBreakSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
    clawConstants.FRICTION_BREAK_PORT);
    rotatingMotor = new WPI_TalonFX(clawConstants.ROTATINGMOTOR_PORT);
  }

  public static ClawAndArm getInstance() {
    if (instance == null) {
      instance = new ClawAndArm();
    }
    return instance;
  }

  public double getPot() {
    return 0.0;//pot.get();
  }

  public void frictionBreakOn() {
   frictionBreakSolenoid.set(true);
  }

   public void frictionBreakOff() {
     frictionBreakSolenoid.set(false);
   }

  public void setRotatingMotor(double speed) {
    double deg = getRotationDegrees();
    
    if (deg <= 5) speed = Math.max(speed, 0);
    else if (deg >= 300) speed = Math.min(speed, 0);

    rotatingMotor.set(speed);
  }

  public void resetRotation() {
    rotatingMotor.setSelectedSensorPosition(0);
  }

  public double getRotationDegrees() {
    return rotatingMotor.getSelectedSensorPosition() * Constants.clawConstants.ROTATIONAL_TICKS2ROT * 360.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("potentiometer", getPot());
    SmartDashboard.putNumber("arm angle", getRotationDegrees());
  }
}

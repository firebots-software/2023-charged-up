// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  // rotating
  private WPI_TalonFX rotatingMotor;
  private AnalogPotentiometer pot;
  private Solenoid frictionBreakSolenoid0, frictionBreakSolenoid1;

  // extending
  private WPI_TalonSRX extendingMotor;
  private DigitalInput topHall, bottomHall;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    pot = new AnalogPotentiometer(ArmConstants.POTENTIOMETER_PORT, ArmConstants.RANGE_OF_MOTION,
        ArmConstants.STARTING_POINT);
    frictionBreakSolenoid0 = new Solenoid(PneumaticsModuleType.REVPH,
        ArmConstants.FRICTION_BREAK_PORT0);
    frictionBreakSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH,
        ArmConstants.FRICTION_BREAK_PORT1);
    rotatingMotor = new WPI_TalonFX(ArmConstants.ROTATINGMOTOR_PORT);

    extendingMotor = new WPI_TalonSRX(ArmConstants.EXTENDINGMOTOR_PORT);
    extendingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    topHall = new DigitalInput(ArmConstants.TOPHALLEFFECT_PORT);
    bottomHall = new DigitalInput(ArmConstants.BOTTOMHALLEFFECT_PORT);
    setRotationWithPot();
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public double _getPotentiometerDegrees() {
    return pot.get();
  }

  public void _frictionBreakOn() {
    if (!_isFrictionBreakOn()) {
      frictionBreakSolenoid0.set(false);
      frictionBreakSolenoid1.set(true);
    }
  }

  public void _frictionBreakOff() {
    if (_isFrictionBreakOn()) {
      frictionBreakSolenoid0.set(true);
      frictionBreakSolenoid1.set(false);
    }
  }

  public boolean _isFrictionBreakOn() {
    return frictionBreakSolenoid1.get();
  }

  public boolean setRotatingMotor(double speed) {
    double deg = _getPotentiometerDegrees();

    if (deg <= -115)
      speed = Math.max(speed, 0);
    else if (deg >= 115)
      speed = Math.min(speed, 0);

    if (Math.abs(deg) <= 60 && !getBottomStatus()) {
      setExtendingMotor(-0.5);
      return false;
    }

    // if moving too slow, turn on friction break and don't move
    if (Math.abs(speed) < ArmConstants.FRICTION_BREAK_DEADBAND) {
      rotatingMotor.set(0);
      _frictionBreakOn();
      return false;
    }

    // else, set the speed and turn the friction break off
    _frictionBreakOff();
    rotatingMotor.set(speed);
    return true;
  }

  public void setRotationWithPot() {
    rotatingMotor.setSelectedSensorPosition(_getPotentiometerDegrees() / (Constants.ArmConstants.ROTATIONAL_TICKS2ROT * 360.0));
  }

  public double getRotationDegrees() {
    return _getPotentiometerDegrees();// rotatingMotor.getSelectedSensorPosition() * Constants.ArmConstants.ROTATIONAL_TICKS2ROT * 360.0;
  }

  // retracting is negative, extending is positive
  public void setExtendingMotor(double speed) {
    if ((Math.abs(speed) < 0.1) ||
    (getBottomStatus() && speed < 0) ||
    (getTopStatus() && speed > 0)) {
      // maintain position
      extendingMotor.set(-0.1);
      return;
    }

    extendingMotor.set(speed);
  }

  public boolean getTopStatus() {
    boolean status = !topHall.get();
    return status;
  }

  public boolean getBottomStatus() {
    boolean status = !bottomHall.get();
    if (status) extendingMotor.setSelectedSensorPosition(0);
    return status;
  }

  public double getArmLength() {
    return _ticksToLength(extendingMotor.getSelectedSensorPosition());
  }

  public double _ticksToLength(double ticks) {
    double r = ticks * ArmConstants.EXTENSION_TICKS2ROT;
    return ((.75 + 0.04*(r+1) ) * Math.PI * r) + 25.65;
  }

  public double getTicks() {
    return extendingMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("potentiometer", _getPotentiometerDegrees());
    SmartDashboard.putNumber("arm angle", getRotationDegrees());

    SmartDashboard.putBoolean("topHalStatus", getTopStatus());
    SmartDashboard.putBoolean("bottomHalStatus", getBottomStatus());
    SmartDashboard.putNumber("ticks", getTicks());


    SmartDashboard.putNumber("extendLength", _ticksToLength(extendingMotor.getSelectedSensorPosition()));

    getBottomStatus();
  }
}

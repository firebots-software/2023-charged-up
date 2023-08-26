// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;
  private boolean canExtend, canExtend2;

  // rotating
  private WPI_TalonFX rotatingMotor;
  private AnalogPotentiometer pot;
  private Solenoid frictionBreakSolenoid;

  private Timer zeroTimer;

  // extending
  private WPI_TalonSRX extendingMotor;
  private PIDController extendingPid;

  private StringLogEntry armLog;

  private boolean isZeroed;

  private boolean stopped;
  private double stopPoint;

  public void unZero() {
    isZeroed = false;
    zeroTimer = new Timer();
    zeroTimer.start();
  }

  /** Creates a new ArmSubsystem. */
  private ArmSubsystem() {
    pot = new AnalogPotentiometer(ArmConstants.POTENTIOMETER_PORT, ArmConstants.RANGE_OF_MOTION,
        ArmConstants.STARTING_POINT);
    frictionBreakSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
        ArmConstants.FRICTION_BREAK_PORT);
    rotatingMotor = new WPI_TalonFX(ArmConstants.ROTATINGMOTOR_PORT);

    extendingMotor = new WPI_TalonSRX(ArmConstants.EXTENDINGMOTOR_PORT);
    extendingMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    extendingPid = new PIDController(ArmConstants.EXTENTION_P, ArmConstants.EXTENTION_I, ArmConstants.EXTENTION_D);

    extendingPid.setTolerance(ArmConstants.pidExtentionToleranceTicks);

    setRotationWithPot();

    armLog = new StringLogEntry(DataLogManager.getLog(), "/log/arm/potLog");

    extendingMotor.setSelectedSensorPosition(0);
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public double _getPotentiometerDegrees() {
    double val = pot.get();
    logArm("pot deg: " + val);
    return val;
  }

  public void _frictionBreakOn() {
    if (!_isFrictionBreakOn())
      frictionBreakSolenoid.set(false);
  }

  public void _frictionBreakOff() {
    if (_isFrictionBreakOn())
      frictionBreakSolenoid.set(true);
  }

  public boolean _isFrictionBreakOn() {
    return !frictionBreakSolenoid.get();
  }

  public boolean setRotatingMotor(double speed) {
    double deg = _getPotentiometerDegrees();
    boolean retracted = getBottomStatus();

    if ((deg <= -ArmConstants.MAX_ROTATION_ANGLE_DEG && !retracted) || (deg <= -ArmConstants.MAX_RETRACTED_DEG)) {
      speed = Math.max(speed, 0);
      logArm("deg " + deg + " too negative, speed to " + speed);
    }
    else if ((deg >= ArmConstants.MAX_ROTATION_ANGLE_DEG && !retracted) || (deg >= ArmConstants.MAX_RETRACTED_DEG)) {
      speed = Math.min(speed, 0);
      logArm("deg " + deg + " too positive, speed to " + speed);
    }

    // Guard 2: if the arm is in between -60 and 60 degrees, we need to retract the arm fully to not extend past 6' 4"
    // TODO: when encoder works, do trig to figure out the actual limit based on the arm length.
    if (Math.abs(deg) <= 60 && !retracted) {
      canExtend = false;
      priorityExtend(-0.5);
      speed = 0; // make sure arm doesn't continue moving
    } else {
      canExtend = true;
    }

    // Guard 3: if the requested speed moves too slow, turn on friction break and don't move
    if (Math.abs(speed) < ArmConstants.FRICTION_BREAK_DEADBAND) {
      rotatingMotor.set(0);
      _frictionBreakOn();
      return false;
    }

    // set the arm to the requested speed and turn the friction break off
    _frictionBreakOff();
    rotatingMotor.set(speed);
    return true;
  }

  public void setRotationWithPot() {
    rotatingMotor.setSelectedSensorPosition(_getPotentiometerDegrees() / (Constants.ArmConstants.ROTATIONAL_TICKS2ROT * 360.0));
  }

  public double getRotationDegrees() {
    return _getPotentiometerDegrees();
  }

  // retracting is negative, extending is positive
  public void setExtendingMotor(double targetTicks) {
    if (canExtend && canExtend2) priorityExtend(targetTicks);
  }

  private long dutycounter;
  private final double DUTY_CYCLE_SECONDS = .25;
  private final double DUTY_CYCLE_PERCENT = .5;

  private void keepArmInPlace() {
    /*
    dutycounter = (dutycounter + 1) % Math.round(DUTY_CYCLE_SECONDS * 50);
    SmartDashboard.putNumber("duty counter", dutycounter);
    if (dutycounter < DUTY_CYCLE_PERCENT * DUTY_CYCLE_SECONDS * 50) {
      extendingMotor.set(-0.2);
      SmartDashboard.putNumber("keep arm speed", -0.4);
    }
    else {
      extendingMotor.set(0);
      SmartDashboard.putNumber("keep arm speed", 0);
    }*/
  
    if (!stopped) {
      stopPoint = getTicks();
      stopped = true;
    }
  
    double dist = stopPoint-getTicks();
    double val = 0.4 * Math.signum(dist);
    SmartDashboard.putNumber("keep arm speed", val);
    if (dist <= 10000) {
      extendingMotor.set(val);
    } else {
      extendingMotor.set(0);
    }
  }

  private void priorityExtend(double speed) {
    double deg = Math.abs(getRotationDegrees());

    if (deg >= ArmConstants.MAX_ROTATION_ANGLE_DEG && speed >= -0.3) { // tucked in
      // cant extend when
      keepArmInPlace();
      return;
    }

    if (deg <= 60 && speed >= 0) { // keep it retracted 
      keepArmInPlace();
      return;
    } 

    if ((Math.abs(speed) < 0.1)) { // speed too low
    //(getBottomStatus() && speed < 0)) { // already retracted
      keepArmInPlace();
      return;
    }

    SmartDashboard.putNumber("retraction speed", speed);


    SmartDashboard.putNumber("keep arm speed", speed);
    stopped = false;
    extendingMotor.set(speed);
  }

  public boolean getBottomStatus() {
    return getTicks() < 5000;
  }

  public double getArmLength() {
    return _ticksToLength(extendingMotor.getSelectedSensorPosition());
  }

  public double _ticksToLength(double ticks) {
    double r = ticks * ArmConstants.EXTENSION_TICKS2ROT;
    return ((.75 + 0.04*(r+1) ) * Math.PI * r) + 25.65;
  }

  public double getTicks() {
    return -extendingMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if (!isZeroed) {
      if (zeroTimer.hasElapsed(5)) {
        isZeroed = true;
        canExtend2 = true;
        extendingMotor.setSelectedSensorPosition(0);
        return;
      }
      priorityExtend(-0.4);
      canExtend2 = false;
      if (extendingMotor.getSupplyCurrent() > ArmConstants.SUPPLY_CURRENT_RETRACTION_THRESHOLD) { 
        logArm("zeroing encoder!");
        extendingMotor.setSelectedSensorPosition(0);
        isZeroed = true;
      }
    } else {
      canExtend2 = true;
    }

    SmartDashboard.putNumber("arm angle", getRotationDegrees());

    SmartDashboard.putBoolean("bottomHalStatus", getBottomStatus());
    SmartDashboard.putNumber("ticks", getTicks());

    SmartDashboard.putNumber("calculated extend length (inches)", getArmLength());
    SmartDashboard.putNumber("supply current", extendingMotor.getSupplyCurrent());
    SmartDashboard.putNumber("motor output voltage", extendingMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("stator current", extendingMotor.getStatorCurrent());
    SmartDashboard.putNumber("bus voltage", extendingMotor.getBusVoltage());
    SmartDashboard.putNumber("output percent", extendingMotor.getMotorOutputPercent());


  }

  private void logArm(String x) {
    if (armLog != null)
      armLog.append(x);
  }
}

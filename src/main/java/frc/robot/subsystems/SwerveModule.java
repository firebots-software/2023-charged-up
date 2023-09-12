package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SwerveModule extends SubsystemBase {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final double absoluteEncoderOffsetRad;
    private final double absoluteDriveEncoderOffset;

    /**
     * Constructor for one Swerve Module
     * @param driveMotorId The port that the drive motor is connected to on the robot
     * @param turningMotorId The port that the turning motor is connected to on the robot
     * @param CANCoderId The port that the CANCoder is connected to on the robot
     * @param driveMotorReversed Sets driving motor to inverse. Inverts direction of a speed controller.
     * @param turningMotorReversed Sets turning motor to inverse. Inverts direction of a speed controller.
     * @param absoluteEncoderOffset The offset of the turning motor. Ensures that the motor is zero'd to the correct position.
     */
    public SwerveModule(int driveMotorId, int turningMotorId, int CANCoderId, boolean driveMotorReversed, boolean turningMotorReversed, double absoluteEncoderOffset) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;

        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
       
        turningEncoder = new CANCoder(CANCoderId);

        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        turningEncoder.configAllSettings(config);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteDriveEncoderOffset = driveMotor.getSelectedSensorPosition();
    }
/**
 * 
 * @return the aboslute position of the encoder after applying the offset (zeros it)
 */
    public double getTurningPosition() {
        return turningEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad;
    }
 /**
  * 
  * @return the velocity of the encoder
  */   
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Gets the current state of the module. 
     * @return a Swerve Module state. The state includes turning motor position and drive motor speed.
     */
    public SwerveModuleState getState() {
        // * 10d because getSelectedSensorVelocity() returns ticks/100ms; 
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * 10d * Constants.ModuleConstants.kDriveEncoderTicks2Meter, new Rotation2d(getTurningPosition()));
    }
/**
 * sets the state module to a desired speed and desired rotation by taking in current value and dividing it by drive constants max speed (4.5106)
 * sets the new rotation value by taking in current by the constant in radians 
 * @param state is a swerve module with the current state of speed and rotation
 */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    /**
     * The position of the drive motor and turning motor.
     * @return The number of meters that the drive motor travelled and the angle of the turning motor in radians.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition((driveMotor.getSelectedSensorPosition()-absoluteDriveEncoderOffset) * Constants.ModuleConstants.kDriveEncoderTicks2Meter, getState().angle);
    }
/** 
 * stops the motor by setting the rotation and speed to 0.
 */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    //private int count = 0;

    @Override
    public void periodic() {
    /*
        count++;
        if (count % 150 == 0) {
            System.out.println("actual m/s: " + getState().speedMetersPerSecond + "\n.\texpected m/s: " + DEBUG_lastms);
            System.out.println("power: " + DEBUG_lastms / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            //System.out.println("distance m: " + getModulePosition().distanceMeters);
            count = 0;
        }
    */
    }

    /**
     * The current position of the drive motor
     * @return The number of ticks that the drive motor has traveled. We are not subtracting Offset here.
     */
    public double getPosition(){
        return driveMotor.getSelectedSensorPosition();
    }
/**
 * takes in a speed and sets the motor value to that
 * @param speed a double value of the desired speed
 */
    public void setMotor(double speed){
        driveMotor.set(speed);
    }

    /**
     * The current position of the drive motor
     * @return The number of ticks that the drive motor has traveled. EncoderOffset is taken into account here.
     */
    public double getDrivingTickValues(){
        return (driveMotor.getSelectedSensorPosition()-absoluteDriveEncoderOffset);
    }
/**
 * setting the drive motor to certain number of ticks
 * @param val desired value of ticks
 */
    public void setDrivingTickValues(double val){
        driveMotor.setSelectedSensorPosition(val);
    }
}


package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

    /**
     * Creates a new SwereModule.
     * @param driveMotorId the driving motor port #
     * @param turningMotorId the roatational motor port #
     * @param CANCoderId the CANCoder port #
     * @param driveMotorReversed if the drive motor should run "backwards" to increase ticks
     * @param turningMotorReversed if the turning motor should run "backwards" to increase ticks 
     * @param absoluteEncoderOffset the encoder offset on the swerve module
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
    }

    // setDesiredState attempts to put the swerve drive into the desired state.
    // this should be run near continuously whenever desired.
    // possible bug: if set once and left to it's own devices, may continue turning forever.
    // solution is to call stop, or update setDesiredState near constantly.
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    // stop sets all motor speeds to zero (stoping any turning or driving).
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    @Override
    public void periodic() {}

    private double getTurningPosition() {
        return turningEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad;
    }

    private SwerveModuleState getState() {
        // * 10d because getSelectedSensorVelocity() returns ticks/100ms; 
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * 10d * Constants.ModuleConstants.kDriveEncoderTicks2Meter, new Rotation2d(getTurningPosition()));
    }

}


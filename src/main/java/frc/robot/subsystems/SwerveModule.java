package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        resetEncoders();
    }

    public double getTurningPosition() {
        return turningEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad;
    }
    
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        //turningEncoder.setPosition(turningEncoder.getAbsolutePosition()-absoluteEncoderOffsetRad);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SwerveModulePosition getModulePosition() {
        SwerveModuleState state = getState();
        return new SwerveModulePosition(state.speedMetersPerSecond, state.angle); // TODO: state.speedMetersPerSecond should be distance traveled, not speed
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    // private int count = 0;

    @Override
    public void periodic() {
        /*
        count++;
        if (count % 100 == 0) {
            System.out.println(debug_turning + " encoder: " + (turningEncoder.getAbsolutePosition()));
            count = 0;
        }
        */
    }
}



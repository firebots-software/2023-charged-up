package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private static SwerveSubsystem instance;

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
             DriveConstants.kFrontRightTurningMotorPort,
             DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
             DriveConstants.kFrontRightDriveEncoderReversed,
             DriveConstants.kFrontRightTurningEncoderReversed,
             DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

     private final SwerveModule backLeft = new SwerveModule(
             DriveConstants.kBackLeftDriveMotorPort,
             DriveConstants.kBackLeftTurningMotorPort,
             DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
             DriveConstants.kBackLeftDriveEncoderReversed,
             DriveConstants.kBackLeftTurningEncoderReversed,
             DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

     private final SwerveModule backRight = new SwerveModule(
             DriveConstants.kBackRightDriveMotorPort,
             DriveConstants.kBackRightTurningMotorPort,
             DriveConstants.kBackRightDriveAbsoluteEncoderPort,
             DriveConstants.kBackRightDriveEncoderReversed,
             DriveConstants.kBackRightTurningEncoderReversed,
             DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
    
    private final edu.wpi.first.math.kinematics.SwerveDriveOdometry odometer = new edu.wpi.first.math.kinematics.SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new edu.wpi.first.math.geometry.Rotation2d(0), 
        getModulePositions()
    );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return 0.0; // Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public edu.wpi.first.math.geometry.Rotation2d getRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(), backRight.getModulePosition()};
    }

    public edu.wpi.first.math.geometry.Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        //odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    } 

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,  DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }
}

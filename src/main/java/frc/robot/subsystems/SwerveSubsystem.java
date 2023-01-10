package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
            5,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    /*
    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
             DriveConstants.kFrontRightTurningMotorPort,
             8,
             DriveConstants.kFrontRightDriveEncoderReversed,
             DriveConstants.kFrontRightTurningEncoderReversed,
             DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
             DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

     private final SwerveModule backLeft = new SwerveModule(
             DriveConstants.kBackLeftDriveMotorPort,
             DriveConstants.kBackLeftTurningMotorPort,
             6,
             DriveConstants.kBackLeftDriveEncoderReversed,
             DriveConstants.kBackLeftTurningEncoderReversed,
             DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
             DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

     private final SwerveModule backRight = new SwerveModule(
             DriveConstants.kBackRightDriveMotorPort,
             DriveConstants.kBackRightTurningMotorPort,
             7,
             DriveConstants.kBackRightDriveEncoderReversed,
             DriveConstants.kBackRightTurningEncoderReversed,
             DriveConstants.kBackRightDriveAbsoluteEncoderPort,
             DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
             DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    */

    // private final PigeonIMU gyro = new PigeonIMU(Constants.Drivetrain.PIGEON_ID);
    
    private final edu.wpi.first.math.kinematics.SwerveDriveOdometry odometer = new edu.wpi.first.math.kinematics.SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new edu.wpi.first.math.geometry.Rotation2d(0));

    public SwerveSubsystem() {
        /*
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                 zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        */
    }

    public void zeroHeading() {
        // gyro.setYaw(0);
    }

    public double getHeading() {
        return 0.0; // Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public edu.wpi.first.math.geometry.Rotation2d getRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromDegrees(getHeading());
    }

    public edu.wpi.first.math.geometry.Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        //odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        odometer.update(getRotation2d(), frontLeft.getState(), frontLeft.getState(), frontLeft.getState(), frontLeft.getState()); 
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();

        /*
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
        */
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,  DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        System.out.println("setting state to " + desiredStates[0].speedMetersPerSecond + " mps, at degrees:" + desiredStates[0].angle.getDegrees());
        frontLeft.setDesiredState(desiredStates[0]);
        /*
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        */
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }
}

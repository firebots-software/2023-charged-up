// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OI {
        public static final int PS4_CONTROLLER_PORT_1 = 3;
        public static final int PS4_CONTROLLER_PORT_2 = 1; // change

        // Buttons on PS4 Controller
        public static final int SQUARE_BUTTON_PORT = 1;
        public static final int X_BUTTON_PORT = 2;
        public static final int CIRCLE_BUTTON_PORT = 3;
        public static final int TRIANGLE_BUTTON_PORT = 4;
        public static final int L1_BUTTON_PORT = 5;
        public static final int R1_BUTTON_PORT = 6;
        public static final int L2_BUTTON_PORT = 7;
        public static final int R2_BUTTON_PORT = 8;
        public static final int PS_SHARE_BUTTON_PORT = 9;
        public static final int OPTIONS_BUTTON_PORT = 10;
        public static final int L3_BUTTON_PORT = 11;
        public static final int R3_BUTTON_PORT = 12;
        public static final int PS_BUTTON_PORT = 13;
        public static final int BIG_BUTTON_PORT = 14;

        // Joystick constants 
        public static final double DEADBAND = 0.05; 
    }

    public static final class Drivetrain {
        public final static int leftFollowerPort = 4; //TODO: change port numbers for new drivetrain
        public final static int leftMasterPort = 3;
        public final static int rightMasterPort = 2;
        public final static int rightFollowerPort = 1;

        public static final double TICKS_PER_METER = 35914.8225; // * (2048 * 8.4) / 0.479;
        public static final double drivetrainTicksPerRev = 2048 * 8.4;
        public static final double drivetrainWheelRadiusMeters = 0.0762; // convert to meters 

        public final static int PIGEON_ID = 8;
        
        public static final double TRACK_WIDTH_METERS = 0.612775;

        public static final DifferentialDriveKinematics kinematics =
                new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static double kMaxSpeedMetersPerSecond = 1;
        public static double kMaxAccelerationMetersPerSecondSquared = 1;

        // ramsete values from wpilib docs
        public static double kRamseteB = 2.0;
        public static double kRamseteZeta = 0.7;
        public static double kPDriveVel = 2.6604;

        public static final double ksVolts = 0.67605;
        public static final double kvVoltSecondsPerMeter = 1.8252;
        public static final double kaVoltSecondsSquaredPerMeter = 0.37032;
		public static final double velocityThreshold = 90; // tune this value

        public static DifferentialDriveVoltageConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(ksVolts,
                                kvVoltSecondsPerMeter,
                                kaVoltSecondsSquaredPerMeter),
                        kinematics,
                        10);

        public static TrajectoryConfig MotionProfilingConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(false);


        public static TrajectoryConfig MotionProfilingConfigReversed = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);
    } 

   public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.8462;
    public static final double kTurningMotorGearRatio = 1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
}
      
    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(29.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(29.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 15;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kBackRightDriveMotorPort = 0;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 14;
        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kBackRightTurningMotorPort = 12;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 5;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 6;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 8;
        public static final int kBackRightDriveAbsoluteEncoderPort = 7;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.270286977291107;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.8269674256443977;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.32366929203271866;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.494094267487526;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
    }
}

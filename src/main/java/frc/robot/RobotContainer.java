// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick ps4_controller1;
  //private Joystick ps4_controller2; 
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.ps4_controller1 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_1);
    //this.ps4_controller2 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_2); 
    
    swerveSubsystem.setModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(0.01, new Rotation2d()),
      new SwerveModuleState(0.01, new Rotation2d()),
      new SwerveModuleState(0.01, new Rotation2d()),
      new SwerveModuleState(0.01, new Rotation2d())
    });


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -ps4_controller1.getRawAxis(1),
                () -> -ps4_controller1.getRawAxis(0),
                () -> -ps4_controller1.getRawAxis(2),
                () -> !ps4_controller1.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));
    // Configure the button bindings
    configureButtonBindings();

    swerveSubsystem.stopModules();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final Trigger damageControl = new JoystickButton(ps4_controller1, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.whileTrue(new ZeroHeadingCmd(swerveSubsystem));

    final Trigger driveDistance = new JoystickButton(ps4_controller1, Constants.OI.X_BUTTON_PORT);
    driveDistance.onTrue(new DriveForDistance());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String trajectoryFileName, boolean shouldResetOdometry) {
    // An ExampleCommand will run in autonomous

      // final Trajectory trajectory = generateTrajectory(waypoints);
      final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 2, 3);
      
      return new InstantCommand(() -> {
        if (shouldResetOdometry) {
          PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
          Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
              initialSample.holonomicRotation);
          swerveSubsystem.resetOdometry(initialPose);
        }
      }).andThen(new PPSwerveControllerCommand(
          trajectory,
          swerveSubsystem::getPose,
          DriveConstants.kDriveKinematics,
          new PIDController(5, 0, 0),
          new PIDController(5, 0, 0),
          new PIDController(0.5, 0, 0),
          swerveSubsystem::setModuleStates,
          true,
          swerveSubsystem));
    }
}

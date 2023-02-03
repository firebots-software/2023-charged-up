// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
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
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.ps4_controller1 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_1);
    //this.ps4_controller2 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_2); 
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -ps4_controller1.getRawAxis(1),
                () -> -ps4_controller1.getRawAxis(0),
                () -> -ps4_controller1.getRawAxis(2),
                () -> !ps4_controller1.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final Trigger damageControl = new JoystickButton(ps4_controller1, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.toggleOnTrue(new ZeroHeadingCmd(swerveSubsystem));
  }

  public static SendableChooser<Command> getAutonChooser(){
    return autonChooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String trajectoryFileName, boolean shouldResetOdometry) {
      //loadPath() will generate swerveModuleStates for an entire "path" drawn in the PathPlanner app
      //  pass in the name of your path file (WITHOUT the .path), max vel (m/s), and max accel (m/s^2)



      final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 1, 2);



      return new InstantCommand(() -> {
        if (shouldResetOdometry) {
          PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
          Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
              initialSample.holonomicRotation);
          swerveSubsystem.resetOdometry(initialPose);
        }
        //the actual command that runs the path
      }).andThen(new FollowPathWithEvents(
        new PPSwerveControllerCommand(
          trajectory,
          swerveSubsystem::getPose,
          DriveConstants.kDriveKinematics,
          new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
          new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
          new PIDController(Constants.PathPlannerConstants.kPTurning, Constants.PathPlannerConstants.kITurning, Constants.PathPlannerConstants.kDTurning),
          swerveSubsystem::setModuleStates,
          true,
          swerveSubsystem),
         trajectory.getMarkers(), 
         Constants.PathPlannerConstants.eventMap));
    }

    

    
}

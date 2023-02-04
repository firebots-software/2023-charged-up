// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commands.EngageCmd;
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

  public final Map<String, Command> eventMap = Map.of(

    //TODO: shmeez aagrims code
    "chargeStationForward", new ChargeStation(swerveSubsystem, 1),
    "chargeStationBackward", new ChargeStation(swerveSubsystem, -1)
);


  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      swerveSubsystem::getPose, 
      swerveSubsystem::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving, Constants.AutonConstants.kDDriving), 
      new PIDConstants(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning, Constants.AutonConstants.kDTurning), 
      swerveSubsystem::setModuleStates, 
      eventMap, 
      swerveSubsystem);

    
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

    
    initializeAutonChooser();


      

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

    final Trigger wobbleWobble = new JoystickButton(ps4_controller1, Constants.OI.TRIANGLE_BUTTON_PORT);
    wobbleWobble.whileTrue(new ChargeStation(swerveSubsystem, 1));
      //new EngageCmd(new PIDController(Constants.DockingConstants.kPDocking, Constants.DockingConstants.kIDocking, Constants.DockingConstants.kDDocking), swerveSubsystem));

    /*
    final PathPlannerTrajectory trajectory = PathPlanner.generatePath(
      new PathConstraints(1, 3), 
      new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
      new PathPoint(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
    );
  
    final Trigger cessina = new JoystickButton(ps4_controller1, Constants.OI.TRIANGLE_BUTTON_PORT);
    cessina.toggleOnTrue(new InstantCommand(() -> {
        PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
        Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
            initialSample.holonomicRotation);
        swerveSubsystem.resetOdometry(initialPose);
      //the actual command that runs the path
    }).andThen(new PPSwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
        new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
        new PIDController(Constants.PathPlannerConstants.kPTurning, Constants.PathPlannerConstants.kITurning, Constants.PathPlannerConstants.kDTurning),
        swerveSubsystem::setModuleStates,
        true,
        swerveSubsystem)
    ));*/

  }

  public void initializeAutonChooser(){
    autonChooser.addOption("topAuton", autoBuilder.fullAuto(AutonPaths.topAuton));
    autonChooser.addOption("chargeStationTest", autoBuilder.fullAuto(AutonPaths.chargeStationTest));
    SmartDashboard.putData(autonChooser);

  }

  public static SendableChooser<Command> getAutonChooser(){
    return autonChooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

      return autonChooser.getSelected();
  }

}
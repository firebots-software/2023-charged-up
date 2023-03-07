// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.RunMotor;
import frc.robot.Constants.DriveConstants;
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.ClosePiston;
import frc.robot.commands.ExtendArmToMax;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.MoveToTargetLeft;
import frc.robot.commands.MoveToTargetRight;
import frc.robot.commands.TogglePiston;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Piston;
import frc.robot.commandGroups.ConePivot;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroArmCmd;
import frc.robot.commands.ZeroHeadingCmd;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // OI
  private Joystick driverPS4;
  private Joystick armJoystick;

  private final ArmSubsystem clawAndArm = ArmSubsystem.getInstance();
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  InstantCommand command;
  ArrayList<PathPoint> points = new ArrayList<>();

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  // PathPlanner
  private final Map<String, Command> eventMap = new HashMap<String, Command>() {{
      put("ChargeStationForward", new ChargeStation(swerveSubsystem, 1));
      put("ChargeStationBackward", new ChargeStation(swerveSubsystem, -1));
      put("MoveToTarget", new MoveToTarget(swerveSubsystem));
      //put("MoveToTargetLeft", new MoveToTargetLeft(swerveSubsystem));
      //put("MoveToTargetRight", new MoveToTargetRight(swerveSubsystem));
      put("ZeroGyro", new ZeroHeadingCmd(swerveSubsystem));
      //put("OpenClaw", new OpenPiston());
      //put("CloseClaw", new ClosePiston());
      put("ArmToHighCubeFront", new ArmToDegree(clawAndArm, ArmConstants.HIGH_CUBE_FRONT_DEG));
      put("ArmToHighConeFront", new ArmToDegree(clawAndArm, ArmConstants.HIGH_CONE_FRONT_DEG));
      put("ArmToGroundBack", new ArmToDegree(clawAndArm, -ArmConstants.MAX_ROTATION_ANGLE_DEG));
      //put("ExtendArmToMax", new ExtendArmToMax(12));
  }};
  

  private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      swerveSubsystem::getPose,
      swerveSubsystem::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving,
          Constants.AutonConstants.kDDriving),
      new PIDConstants(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning,
          Constants.AutonConstants.kDTurning),
      swerveSubsystem::setModuleStates,
      eventMap,
      true,
      swerveSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driverPS4 = new Joystick(Constants.OI.DRIVER_PS4_PORT);
    this.armJoystick = new Joystick(Constants.OI.ARM_JOYSTICK_PORT); 
    
    swerveSubsystem.resetEncoders();
    swerveSubsystem.zeroHeading();

    
    // Configure the button bindings
    configureButtonBindings();

    initializeAutonChooser();
  }

  private void configureButtonBindings() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverPS4.getRawAxis(1),
        () -> -driverPS4.getRawAxis(0),
        () -> -driverPS4.getRawAxis(2),
        () -> (driverPS4.getRawAxis(4) + 1d) / 2d,

        () -> !driverPS4.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));

        
    clawAndArm.setDefaultCommand(new ArmJoystickCmd(
      () -> armJoystick.getRawAxis(0) * 0.2,
      () -> -armJoystick.getRawAxis(1) * 0.5));
      

    final Trigger armToDegree = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    armToDegree.whileTrue(new ArmToDegree(clawAndArm, 90));
    
    final Trigger damageControl = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.toggleOnTrue(new ZeroHeadingCmd(swerveSubsystem));

    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax),
        new ArrayList<>() {
          {
            add(new PathPoint(new Translation2d(0, 0), new Rotation2d(0), new Rotation2d(0)));
            add(new PathPoint(new Translation2d(1, 0), new Rotation2d(0), new Rotation2d(Math.PI / 2.0)));
          }
        });

    final Trigger followPath = new JoystickButton(driverPS4, Constants.OI.TRIANGLE_BUTTON_PORT);
    followPath.toggleOnTrue(new InstantCommand(() -> {
      PathPlannerState initial = (PathPlannerState) traj.sample(0);
      Pose2d initialPose = new Pose2d(initial.poseMeters.getTranslation(), initial.holonomicRotation);
      swerveSubsystem.resetOdometry(initialPose);
    }).andThen(autoBuilder.followPath(traj)));

    final Trigger conePivot = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    conePivot.whileTrue(new ConePivot(swerveSubsystem, 0.7, true));

    //final Trigger mtt = new JoystickButton(driverPS4, Constants.OI.R1_BUTTON_PORT);
    //mtt.toggleOnTrue(new MoveToTarget(swerveSubsystem));

    final Trigger clawToggle = new JoystickButton(armJoystick, 1);
    clawToggle.onTrue(new TogglePiston());

    /*
    final Trigger closePiston = new JoystickButton(driverPS4, Constants.OI.BIG_BUTTON_PORT);
    closePiston.toggleOnTrue(new ClosePiston());

    final Trigger openPiston = new JoystickButton(driverPS4, Constants.OI.PS_BUTTON_PORT);
    openPiston.toggleOnTrue(new OpenPiston());
    */
  }
  // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
  // this value as a decimal to multiply and control the speed of the robot.

  public void initializeAutonChooser() {

    autonChooser.addOption("topAuton", autoBuilder.fullAuto(AutonPaths.topAuton));
    autonChooser.addOption("middleAuton", autoBuilder.fullAuto(AutonPaths.middleAuton));
    autonChooser.addOption("bottomAuton", autoBuilder.fullAuto(AutonPaths.bottomAuton));
    autonChooser.addOption("test", autoBuilder.fullAuto(AutonPaths.test));

    SmartDashboard.putData(autonChooser);
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
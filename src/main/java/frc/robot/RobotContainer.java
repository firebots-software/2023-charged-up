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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.RunMotor;
import frc.robot.Constants.DriveConstants;
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.LimpArm;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.ToggleClaw;
import frc.robot.commandGroups.ConePivot;
import frc.robot.commandGroups.MoveAndScore;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // OI
  private Joystick driverPS4;
  private Joystick armJoystick;
  private Joystick numpad;

  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final ClawSubsystem claw = ClawSubsystem.getInstance();
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  InstantCommand command;
  ArrayList<PathPoint> points = new ArrayList<>();

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  // PathPlanner
  private final Map<String, Command> eventMap = new HashMap<String, Command>() {{
      put("ChargeStationForward", new ChargeStation(swerveSubsystem, 1));
      put("ChargeStationBackward", new ChargeStation(swerveSubsystem, -1));
      //put("MoveToTarget", new MoveToTag(swerveSubsystem));
      //put("MoveToTargetLeft", new MoveToTag(-1, swerveSubsystem));
      //put("MoveToTargetRight", new MoveToTag(1, swerveSubsystem));
      //put("OpenClaw", new ToggleClaw(true, claw));
      //put("CloseClaw", new ToggleClaw(false, claw));
      //  put("ArmToGroundBack", new ArmToDegree(arm, -ArmConstants.MAX_ROTATION_ANGLE_DEG));
      //put("ArmToHighCone", new ArmToDegree(arm, ArmConstants.HIGH_CONE_FRONT_DEG));
      put("ArmToMidCube", new MoveAndScore(0, 1, swerveSubsystem, arm, claw));
      //put("ExtendArmToMax", new ExtendToCmd(arm));
      put("RetractArmToMin", new RetractArmCmd(arm));

      // put("MoveToTargetLeft", new MoveToTargetLeft(swerveSubsystem));
      //put("MoveToTargetRight", new MoveToTargetRight(swerveSubsystem));
      // put("ZeroGyro", new ZeroHeadingCmd(swerveSubsystem));
      // put("OpenClaw", new OpenPiston());
      // put("CloseClaw", new ClosePiston());
      // put("ArmToHighCubeFront", new ArmToDegree(arm, ArmConstants.HIGH_CUBE_FRONT_DEG));
      // put("ArmToHighConeFront", new ArmToDegree(arm, ArmConstants.HIGH_CONE_FRONT_DEG));
      // put("ArmToGroundBack", new ArmToDegree(arm, -ArmConstants.MAX_ROTATION_ANGLE_DEG));
      
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
    this.numpad = new Joystick(Constants.OI.NUMPAD_PORT);

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
        () -> driverPS4.getRawAxis(4) > -0.75 ? 0.80 : (driverPS4.getRawAxis(3) > -0.75 ? 0.15 : 0.40),

        () -> !driverPS4.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));

    arm.setDefaultCommand(new ArmJoystickCmd(
        () -> armJoystick.getRawAxis(0) * 0.4,
        () -> -armJoystick.getRawAxis(1) * 0.5));
    
    for (int button = 1; button < 10; button++) {
      new JoystickButton(numpad, button).whileTrue(new MoveAndScore(((button-1) % 3) - 1, (button-1) / 3, true, swerveSubsystem, arm, claw));
    }

    final Trigger armToDegree = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    armToDegree.whileTrue(new ChargeStation(swerveSubsystem, 1));

    final Trigger ark = new JoystickButton(driverPS4, Constants.OI.X_BUTTON_PORT);
    ark.whileTrue(new MoveToTag(swerveSubsystem));

    final Trigger damageControl = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.toggleOnTrue(new ZeroHeadingCmd(swerveSubsystem));

    final Trigger limpArm = new JoystickButton(driverPS4, Constants.OI.PS_SHARE_BUTTON_PORT);
    limpArm.whileTrue(new LimpArm(arm));

    // PathPlannerTrajectory traj = PathPlanner.generatePath(
    //     new PathConstraints(Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax),
    //     new ArrayList<>() {
    //       {
    //         add(new PathPoint(new Translation2d(0, 0), new Rotation2d(0), new Rotation2d(0)));
    //         add(new PathPoint(new Translation2d(1, 0), new Rotation2d(0), new Rotation2d(Math.PI / 2.0)));
    //       }
    //     });

    // final Trigger followPath = new JoystickButton(driverPS4, Constants.OI.TRIANGLE_BUTTON_PORT);
    // followPath.toggleOnTrue(new InstantCommand(() -> {
    //   PathPlannerState initial = (PathPlannerState) traj.sample(0);
    //   Pose2d initialPose = new Pose2d(initial.poseMeters.getTranslation(), initial.holonomicRotation);
    //   swerveSubsystem.resetOdometry(initialPose);
    // }).andThen(autoBuilder.followPath(traj)));

    // final Trigger conePivot = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    // conePivot.whileTrue(new ConePivot(swerveSubsystem, 0.7, true));

    // final Trigger mtt = new JoystickButton(driverPS4, Constants.OI.R1_BUTTON_PORT);
    // mtt.toggleOnTrue(new MoveToTag(swerveSubsystem));

    final Trigger clawToggle = new JoystickButton(armJoystick, 1);
    clawToggle.onTrue(new ToggleClaw(claw));
  }
  // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
  // this value as a decimal to multiply and control the speed of the robot.

  public void initializeAutonChooser() {

    autonChooser.addOption("oranjeUp", makeAuton((AutonPaths.oranjeup)));
    autonChooser.addOption("oranjeMid", makeAuton((AutonPaths.oranjemid)));
    

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

  public Command makeAuton(List<PathPlannerTrajectory> ppt1){
    
    return new InstantCommand(() -> {
      PathPlannerState initial1 = (PathPlannerState) ppt1.get(0).sample(0);
      Pose2d initial1Pose = new Pose2d(initial1.poseMeters.getTranslation(), initial1.holonomicRotation);
      swerveSubsystem.resetOdometry(initial1Pose);
    }).andThen(autoBuilder.fullAuto(ppt1));
  }

  public Command makeAuton(List<PathPlannerTrajectory> ppt1, List<PathPlannerTrajectory> ppt2){
    
    return new InstantCommand(() -> {
      PathPlannerState initial1 = (PathPlannerState) ppt1.get(0).sample(0);
      Pose2d initial1Pose = new Pose2d(initial1.poseMeters.getTranslation(), initial1.holonomicRotation);
      swerveSubsystem.resetOdometry(initial1Pose);
    }).andThen(autoBuilder.fullAuto(ppt1))
    .andThen(new InstantCommand(() -> {
      PathPlannerState initial2 = (PathPlannerState) ppt2.get(0).sample(0);
      Pose2d initial2Pose = new Pose2d(initial2.poseMeters.getTranslation(), initial2.holonomicRotation);
      swerveSubsystem.resetOdometry(initial2Pose);
    })).andThen(autoBuilder.fullAuto(ppt2));
  }

  // public Command makeAuton(List<List<PathPlannerTrajectory>> ppts) {
  //   Command current = new InstantCommand();

  //   for (int i = 0; i < ppts.size(); i++) {
  //     current = current.andThen(new InstantCommand(() -> {
  //       PathPlannerState initial1 = (PathPlannerState) ppts.get(i).get(0).sample(0);
  //       Pose2d initial1Pose = new Pose2d(initial1.poseMeters.getTranslation(), initial1.holonomicRotation);
  //       swerveSubsystem.resetOdometry(initial1Pose);
  //     })).andThen(autoBuilder.fullAuto(ppts.get(i)));
  //   }
  //   return current;
  // }
  

  
}
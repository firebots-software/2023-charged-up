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
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DockingConstants;
//import frc.robot.commands.RunMotor;
import frc.robot.Constants.DriveConstants;
import frc.robot.commandGroups.ArmToGround;
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.LevelCmd;
import frc.robot.commands.LimpArm;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.ToggleClaw;
import frc.robot.commandGroups.ConePivot;
import frc.robot.commandGroups.MoveAndScore;
import frc.robot.commandGroups.MoveToCubeAndExtend;
import frc.robot.commandGroups.MoveToTargetAndExtend;
import frc.robot.commandGroups.PickupFromGround;
import frc.robot.commandGroups.PickupObjectFromHeight;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // OI
  private Joystick driverPS4;
  private Joystick armJoystick;
  private Joystick numpad;

  
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final ClawSubsystem claw = ClawSubsystem.getInstance();
  // PathPlanner
  private final Map<String, Command> eventMap = new HashMap<String, Command>() {{
      put("ChargeStationForward", new ChargeStation(swerveSubsystem, DockingConstants.DOCKING_SPEED));
      put("ChargeStationBackward", new ChargeStation(swerveSubsystem, -DockingConstants.DOCKING_SPEED));
      put("MoveToTarget", new MoveToTag(swerveSubsystem));
      // put("MoveToTargetLeft", new MoveToTag(MoveToTag.LEFT, swerveSubsystem));
      // put("MoveToTargetRight", new MoveToTag(MoveToTag.RIGHT, swerveSubsystem));
      put("ScoreMidCube", new MoveAndScore(MoveAndScore.MIDDLE_POS, MoveAndScore.MID_LEVEL, swerveSubsystem, arm, claw, true));
      put("ScoreHighCube", new MoveAndScore(MoveAndScore.MIDDLE_POS, MoveAndScore.HIGH_LEVEL, swerveSubsystem, arm, claw, true));
      put("ScoreHighCone", new MoveAndScore(MoveAndScore.RIGHT_POS, MoveAndScore.HIGH_LEVEL, swerveSubsystem, arm, claw, true));
      put("RetractArmToMin", new RetractArmCmd(arm));
      put("ArmToGroundBack", new ArmToGround(() -> true, arm, true));
      put("ExtendArmToMax", new JankArmToTicks(304433, arm));
      put("ToggleClaw", new ToggleClaw(claw));
      put("MoveToCubeAndExtend", new MoveToCubeAndExtend(swerveSubsystem, arm));
      put("MoveToTargetAndExtend", new MoveToTargetAndExtend(swerveSubsystem, arm));
      put("TuckArm", new ArmToDegree(arm, ArmConstants.MAX_RETRACTED_DEG));
      put("CloseClaw", new CloseClaw(claw));
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
        () -> driverPS4.getRawAxis(4) > -0.75 ? 1 : (driverPS4.getRawAxis(3) > -0.75 ? 0.15 : 0.50),

        () -> !driverPS4.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));

    arm.setDefaultCommand(new ArmJoystickCmd(
        () -> armJoystick.getRawAxis(0) * 0.5,
        () -> -armJoystick.getRawAxis(1) * 0.5));
    
    for (int button = 1; button < 10; button++) {
      new JoystickButton(numpad, button).whileTrue(new MoveAndScore(((button-1) % 3) - 1, (button-1) / 3, swerveSubsystem, arm, claw));
    }

    

    final Trigger armToDegree = new JoystickButton(driverPS4, Constants.OI.X_BUTTON_PORT);
    armToDegree.whileTrue(new PickupFromGround(() -> arm.getRotationDegrees() > 0, arm, claw));
    
    final Trigger tri = new JoystickButton(driverPS4, Constants.OI.TRIANGLE_BUTTON_PORT);
    tri.whileTrue(new PickupObjectFromHeight());

    final Trigger to90 = new JoystickButton(driverPS4, Constants.OI.R1_BUTTON_PORT);
    to90.whileTrue(new ArmToDegree(arm, () -> 90d));

    final Trigger damageControl = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.onTrue(new ZeroHeadingCmd(swerveSubsystem));

    final Trigger thing = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    thing.whileTrue(new ChargeStation(swerveSubsystem, -2.0));

    final Trigger limpArm = new JoystickButton(driverPS4, Constants.OI.PS_SHARE_BUTTON_PORT);
    limpArm.whileTrue(new LimpArm(arm));

    final Trigger clawToggle = new JoystickButton(armJoystick, 1);
    clawToggle.onTrue(new ToggleClaw(claw));

    final Trigger goUp = new JoystickButton(armJoystick, 6);
    goUp.onTrue(new RetractArmCmd(arm).andThen(new ArmToDegree(arm, 0)));

  }
  // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
  // this value as a decimal to multiply and control the speed of the robot.

  public void initializeAutonChooser() {

    autonChooser.setDefaultOption("just score", new RetractArmCmd(arm).andThen(new MoveAndScore(0, 1, swerveSubsystem, arm, claw, true)));
    autonChooser.addOption("complexTopAuton", makeAuton(AutonPaths.complexTopAuton));
    autonChooser.addOption("topAuton", makeAuton(AutonPaths.topAuton));
    autonChooser.addOption("midAuton", makeAuton(AutonPaths.midAuton));
    autonChooser.addOption("bottomAuton", makeAuton(AutonPaths.bottomAuton));
    autonChooser.addOption("topAutonNoCharge", makeAuton(AutonPaths.topAutonNoCharge));
    autonChooser.addOption("bottomAutonNoCharge", makeAuton(AutonPaths.bottomAutonNoCharge));

    SmartDashboard.putData("auton chooser", autonChooser);

  }

  /**
   * <p> Commands in the eventMap that change the robot position (including rotation)
   while a PathPlannerTrajectory is running accumulate PID error. PathPlanner tries to compensate by
   quickly correcting the PID error and then running the scheduled path, often overriding the maximum
   velocity in order to run on time. 

   <p> An example of this is {@link MoveToTag}. This command will find an AprilTag during autonomous
   and try to align the robot for scoring. However, if we try to align to the tag, score, and try running
   a trajectory that moves to a game piece, the error accumulates over time.

   <p> Ending each trajectory with the command that changes the robot position (e.g. MoveToTag),
   and then chaining each trajectory, will prevent any PID error from accumulating.

   * @param ppts A List comprised of a List of PathPlannerTrajectories, loaded in {@link AutonPaths}
   * @return A full autonomous command made by autoBuilder
   */

  public Command makeAuton(List<List<PathPlannerTrajectory>> ppts) {
    
    // Instantiate the auton path
    Command auton = new InstantCommand();
    
    // loop through every trajectory (each ending with a translational and/or rotational swerve cmd outside of pathplanner)
      for (List<PathPlannerTrajectory> ppt : ppts) {
      
        if (ppt != null){ // check if the trajectory exists, or if the filename was misspelled

          auton = auton.andThen(
          //reset odometry to the initial pose of each trajectory
          new InstantCommand(() -> {
            PathPlannerState initial = (PathPlannerState) ppt.get(0).sample(0);
            Pose2d initialPose = new Pose2d(initial.poseMeters.getTranslation(), initial.holonomicRotation);
            swerveSubsystem.resetOdometry(initialPose);
          })).andThen(
            // build the trajectory
            autoBuilder.fullAuto(ppt));

        }
       
        else {
          // reports null trajectory, and prevents any path from running
          System.out.println("************** path is null! *************");
          return new InstantCommand();
        }

      } 

      
      
    
    return auton;

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autonChooser.getSelected();
  }

  

  

  public void printAuton() {
    
    SmartDashboard.putString("selected auton", getAutonomousCommand().getName());
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
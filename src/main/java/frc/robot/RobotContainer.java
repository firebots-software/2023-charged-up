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
import frc.robot.commands.MoveToTag;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commandGroups.ConePivot;
import frc.robot.commandGroups.MoveAndScore;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.commands.claw.Intake;
import frc.robot.commands.claw.Outtake;
import frc.robot.commands.claw.ToggleClaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.NewClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // OI
  private Joystick driverPS4;
  private Joystick armJoystick;
  private Joystick numpad;

  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final ClawSubsystem claw = ClawSubsystem.getInstance();
  private final NewClawSubsystem newClaw = NewClawSubsystem.getInstance();
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  InstantCommand command;
  ArrayList<PathPoint> points = new ArrayList<>();

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  // PathPlanner
  private final Map<String, Command> eventMap = new HashMap<String, Command>() {{
      put("ChargeStationForward", new ChargeStation(swerveSubsystem, 1));
      put("ChargeStationBackward", new ChargeStation(swerveSubsystem, -1));
      put("MoveToTag", new MoveToTag(swerveSubsystem));
      //put("MoveToTargetLeft", new MoveToTag(-1, swerveSubsystem));
      //put("MoveToTargetRight", new MoveToTag(1, swerveSubsystem));
      //put("OpenClaw", new ToggleClaw(true, claw));
      //put("CloseClaw", new ToggleClaw(false, claw));
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
      new JoystickButton(numpad, button).whileTrue(new MoveAndScore(((button-1) % 3) - 1, (button-1) / 3, swerveSubsystem, arm, claw));
    }

    final Trigger intake = new JoystickButton(armJoystick, 1);
    intake.whileTrue(new Intake(newClaw));

    final Trigger outtake = new JoystickButton(armJoystick, 2);
    outtake.whileTrue(new Outtake(newClaw));

    final Trigger damageControl = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.onTrue(new ZeroHeadingCmd(swerveSubsystem));


    final Trigger clawToggle = new JoystickButton(armJoystick, 1);
    clawToggle.onTrue(new ToggleClaw(claw));

    final Trigger goUp = new JoystickButton(armJoystick, 6);
    goUp.onTrue(new RetractArmCmd(arm).andThen(new ArmToDegree(arm, 0)));
  }
  // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
  // this value as a decimal to multiply and control the speed of the robot.

  public void initializeAutonChooser() {

    autonChooser.addOption(
      "my auton", 
      makeAuton(AutonPaths.testTopAuton)
      );
    
    SmartDashboard.putData(autonChooser);
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
    
    // loop through every trajectory (each ending with a translational and/or rotational command outside of pathplanner)
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
      // reports null trajectory, and prevents any path from running
      else {
        System.out.println("************** a pathplannertrajectory is null! *************");
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

}
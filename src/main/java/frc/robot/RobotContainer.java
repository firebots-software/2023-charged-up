// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.MoveToTag;
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

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final ClawSubsystem claw = ClawSubsystem.getInstance();

  // PathPlanner
  private final Map<String, Command> eventMap = Map.of(
      "chargeStationForward", new ChargeStation(swerveSubsystem, 1),
      "chargeStationBackward", new ChargeStation(swerveSubsystem, -1),
      "moveToTarget", new MoveToTag(swerveSubsystem),
      "zeroGyro", new ZeroHeadingCmd(swerveSubsystem));
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

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
      true, // TODO: maybe this is messing up auton?
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

    configureButtonBindings();

    initializeAutonChooser();
  }

  private void configureButtonBindings() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverPS4.getRawAxis(1),
        () -> -driverPS4.getRawAxis(0),
        () -> -driverPS4.getRawAxis(2),

        // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
        // this value as a decimal to multiply and control the speed of the robot.
        () -> (driverPS4.getRawAxis(4) + 1d) / 2d,

        () -> !driverPS4.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));

    arm.setDefaultCommand(new ArmJoystickCmd(
        () -> armJoystick.getRawAxis(0) * 0.2,
        () -> -armJoystick.getRawAxis(1) * 0.5));
    
    for (int button = 1; button < 10; button++) {
      new JoystickButton(numpad, button).onTrue(new MoveAndScore(((button-1) % 3) - 1, (button-1) / 3, true, swerveSubsystem, arm, claw));
    }

    // final Trigger armToDegree = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    // armToDegree.whileTrue(new ArmToDegree(arm, 90));

    final Trigger damageControl = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.toggleOnTrue(new ZeroHeadingCmd(swerveSubsystem));

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

    // final Trigger clawToggle = new JoystickButton(armJoystick, 1);
    // clawToggle.onTrue(new TogglePiston(claw));
  }

  public void initializeAutonChooser() {

    autonChooser.addOption("topAuton", autoBuilder.fullAuto(AutonPaths.topAuton));
    autonChooser.addOption("middleAuton", autoBuilder.fullAuto(AutonPaths.middleAuton));
    autonChooser.addOption("bottomAuton", autoBuilder.fullAuto(AutonPaths.bottomAuton));
    autonChooser.addOption("test", autoBuilder.fullAuto(AutonPaths.test));
    SmartDashboard.putData(autonChooser);
  }

  public static SendableChooser<Command> getAutonChooser() {
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
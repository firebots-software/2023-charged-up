// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commandGroups.ChargeStation;
import frc.robot.commandGroups.ConePivot;
import frc.robot.commands.ClosePiston;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.OpenPiston;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // OI
  private Joystick ps4_controller1;
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  // PathPlanner
  private final Map<String, Command> eventMap = Map.of(
      "chargeStationForward", new ChargeStation(swerveSubsystem, 1),
      "chargeStationBackward", new ChargeStation(swerveSubsystem, -1));
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
      swerveSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // OI
    this.ps4_controller1 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_1);

    // Configure the button bindings
    configureButtonBindings();

    initializeAutonChooser();
  }

  private void configureButtonBindings() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -ps4_controller1.getRawAxis(1),
        () -> -ps4_controller1.getRawAxis(0),
        () -> -ps4_controller1.getRawAxis(2),

        // Changing the R2 axis range from [-1, 1] to [0, 1] because we are using
        // this value as a decimal to multiply and control the speed of the robot.
        () -> (ps4_controller1.getRawAxis(4) + 1d) / 2d,

        () -> !ps4_controller1.getRawButton(Constants.OI.SQUARE_BUTTON_PORT)));

    final Trigger damageControl = new JoystickButton(ps4_controller1, Constants.OI.CIRCLE_BUTTON_PORT);
    damageControl.toggleOnTrue(new ZeroHeadingCmd(swerveSubsystem));

    final Trigger wobbleWobble = new JoystickButton(ps4_controller1, Constants.OI.TRIANGLE_BUTTON_PORT);
    wobbleWobble.whileTrue(new ChargeStation(swerveSubsystem, 1));

    final Trigger conePivot = new JoystickButton(ps4_controller1, Constants.OI.SQUARE_BUTTON_PORT);
    conePivot.whileTrue(new ConePivot(swerveSubsystem, 0.7, true));

    final Trigger followPath = new JoystickButton(ps4_controller1, Constants.OI.R1_BUTTON_PORT);
    followPath.toggleOnTrue(new MoveToTarget(swerveSubsystem));

    final Trigger closePiston = new JoystickButton(ps4_controller1, Constants.OI.BIG_BUTTON_PORT);
    closePiston.toggleOnTrue(new ClosePiston());

    final Trigger openPiston = new JoystickButton(ps4_controller1, Constants.OI.PS_BUTTON_PORT);
    openPiston.toggleOnTrue(new OpenPiston());
  }

  public void initializeAutonChooser() {
    autonChooser.addOption("topAuton", autoBuilder.fullAuto(AutonPaths.topAuton));
    autonChooser.addOption("chargeStationTest", autoBuilder.fullAuto(AutonPaths.chargeStationTest));
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
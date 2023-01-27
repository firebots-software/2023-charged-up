// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PortNumCmd;
import frc.robot.commands.RunForDistance;
import frc.robot.commands.WheelOrientationCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.subsystems.SingleMotor;
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
  private SingleMotor dummyMotor;
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.ps4_controller1 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_1);
    dummyMotor = new SingleMotor(-1);

    Supplier<Boolean> incrementer = () -> ps4_controller1.getRawButtonPressed(Constants.OI.R1_BUTTON_PORT); 
    Supplier<Boolean> decrementer = () -> ps4_controller1.getRawButtonPressed(Constants.OI.L1_BUTTON_PORT); 

    /*
    dummyMotor.setDefaultCommand(new PortNumCmd(
      incrementer, 
      decrementer, 
      () -> ps4_controller1.getRawButtonPressed(Constants.OI.X_BUTTON_PORT), 
      () -> ps4_controller1.getRawButtonReleased(Constants.OI.X_BUTTON_PORT),
      dummyMotor
    ));
    */

    /*
    dummyMotor.setDefaultCommand(new WheelOrientationCmd(
      new ArrayList<WheelOrientationCmd.TestModule>(){{
        add(new WheelOrientationCmd.TestModule(
          Constants.DriveConstants.kFrontLeftDriveMotorPort, 
          Constants.DriveConstants.kFrontLeftTurningMotorPort,
          Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort)
        );
        add(new WheelOrientationCmd.TestModule(
          Constants.DriveConstants.kFrontRightDriveMotorPort, 
          Constants.DriveConstants.kFrontRightTurningMotorPort,
          Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort)
        );
        add(new WheelOrientationCmd.TestModule(
          Constants.DriveConstants.kBackLeftDriveMotorPort, 
          Constants.DriveConstants.kBackLeftTurningMotorPort,
          Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort)
        );
        add(new WheelOrientationCmd.TestModule(
          Constants.DriveConstants.kBackRightDriveMotorPort, 
          Constants.DriveConstants.kBackRightTurningMotorPort,
          Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort)
        );
      }}, 
      incrementer, 
      decrementer, 
      () -> -ps4_controller1.getRawAxis(0), 
      () -> ps4_controller1.getRawButtonPressed(Constants.OI.X_BUTTON_PORT),
      () -> ps4_controller1.getRawButton(Constants.OI.CIRCLE_BUTTON_PORT),
      dummyMotor
    ));
    */

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

    final Trigger moveMeters = new JoystickButton(ps4_controller1, Constants.OI.TRIANGLE_BUTTON_PORT);
    moveMeters.toggleOnTrue(new RunForDistance(1, () -> ps4_controller1.getRawButtonPressed(Constants.OI.SQUARE_BUTTON_PORT), swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
      return null;
  }
}

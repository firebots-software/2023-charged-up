// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.FrictionBreakOn;
//import frc.robot.commands.FrictionBreakOn;
import frc.robot.commands.RunMotor;
import frc.robot.commands.RunMotor3;
//import frc.robot.commands.RunMotor;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroArmCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.subsystems.ClawAndArm;
import frc.robot.subsystems.ClawAndArm;
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
  private final ClawAndArm clawAndArm = ClawAndArm.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.ps4_controller1 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_1);
    //this.ps4_controller2 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_2); 
    
    clawAndArm.setDefaultCommand(new ArmJoystickCmd(
      () -> ps4_controller1.getRawAxis(0) * 0.1
    ));

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
    // final Trigger damageControl = new JoystickButton(ps4_controller1, Constants.OI.CIRCLE_BUTTON_PORT);
    // damageControl.toggleOnTrue(new ZeroHeadingCmd(swerveSubsystem));
     final Trigger damageControl = new JoystickButton(ps4_controller1, Constants.OI.TRIANGLE_BUTTON_PORT);
     damageControl.whileTrue(new ZeroArmCmd());
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

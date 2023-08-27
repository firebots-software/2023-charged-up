// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.SampleParallelCG;
import frc.robot.commandGroups.SampleSequentialCG;
import frc.robot.commands.SampleCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // Subsystems
  private SwerveSubsystem swerve;

  // OI
  private Joystick driverPS4;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driverPS4 = new Joystick(Constants.OI.DRIVER_PS4_PORT);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // when no command is running that uses the swerve subsystem [defined by addRequirements() call in commands]
    // then run the default command supplied (SampleCmd in this case)
    swerve.setDefaultCommand(new SampleCmd(swerve));

    // when circle is pressed, run SampleCmd until the command reports it's done
    final Trigger runSample = new JoystickButton(driverPS4, Constants.OI.CIRCLE_BUTTON_PORT);
    runSample.onTrue(new SampleCmd(swerve));

    // when x is pressed, run SampleParallelCG until x is released OR the command reports it's done
    final Trigger runParallel = new JoystickButton(driverPS4, Constants.OI.X_BUTTON_PORT);
    runParallel.whileTrue(new SampleParallelCG(swerve));

    // when square is pressed, run SampleSequentialCG until x is pressed again OR the command reports it's done
    final Trigger runSequential = new JoystickButton(driverPS4, Constants.OI.SQUARE_BUTTON_PORT);
    runSequential.toggleOnTrue(new SampleSequentialCG(swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(1);
  }
}
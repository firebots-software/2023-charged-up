// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.MoveToTargetLeft;
import frc.robot.commands.MoveToTargetLeftPickup;
import frc.robot.commands.RetractArmCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupObjectFromHeight extends SequentialCommandGroup {
  ArmSubsystem arm = ArmSubsystem.getInstance();
  SwerveSubsystem ss = SwerveSubsystem.getInstance();
  /** Creates a new PickupObjectFromHeight. */
  public PickupObjectFromHeight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RetractArmCmd(arm), new ArmToDegree(arm, Constants.ArmConstants.SHELF_PICKUP), new MoveToTargetLeftPickup(ss), new JankArmToTicks(7000, arm)); // TODO: FIND ACTUAL VALUES
  }
}
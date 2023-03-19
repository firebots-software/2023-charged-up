// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.MoveToTag;

public class MoveToTargetLeftPickup extends MoveToTag {
  public MoveToTargetLeftPickup(SwerveSubsystem swerve) {
    super(-1.13, -1.17475, swerve);
  }
}

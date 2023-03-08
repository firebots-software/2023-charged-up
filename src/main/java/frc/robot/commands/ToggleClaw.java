// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawSubsystem;

public class ToggleClaw extends InstantCommand {
  public ToggleClaw(ClawSubsystem p) {
    super(() -> {p.toggle();}, p);
  }

  public ToggleClaw(boolean open, ClawSubsystem p) {
    super(() -> {if (open) { p.open(); } else { p.close(); } }, p);
  }
}
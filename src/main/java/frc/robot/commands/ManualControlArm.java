// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawAndArm;

public class ManualControlArm extends CommandBase {
  private ClawAndArm clawAndArm;

  private double x, y;
  boolean extending = true;
  /** Creates a new ManualControlArm. */
  public ManualControlArm(double xAxis, double yAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    xAxis = x;
    yAxis = y;
    clawAndArm = ClawAndArm.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(y) > Math.abs(x)) {
      extending = true;
    } else {
      extending = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extending) {
      clawAndArm.setExtendingMotor(y);
    } else {
      clawAndArm.setRotatingMotor(x);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

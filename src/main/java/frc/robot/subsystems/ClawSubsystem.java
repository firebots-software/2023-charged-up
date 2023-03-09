// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ClawSubsystem extends SubsystemBase {
  private Solenoid left, right;
  private static ClawSubsystem instance;

  /** Creates a new Piston. */
  public ClawSubsystem() {
    left = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.CLAW_SOLENOID_PORT0);
    right = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.CLAW_SOLENOID_PORT1);
  }

  public static ClawSubsystem getInstance() {
    if (instance == null) {
      instance = new ClawSubsystem();
    }

    return instance;
  }

  public void close() {
    left.set(true);
    right.set(false);
  }

  public void open() {
    left.set(false);
    right.set(true);
  }

  public void toggle() {
    left.set(!left.get());
    right.set(!right.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

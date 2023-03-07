// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ClawSubsystem extends SubsystemBase {
  private Solenoid pistons;
  private static ClawSubsystem instance;

  /** Creates a new Piston. */
  public ClawSubsystem() {
    pistons = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.CLAW_SOLENOID_PORT);
  }

  public static ClawSubsystem getInstance() {
    if (instance == null) {
      instance = new ClawSubsystem();
    }

    return instance;
  }

  public void close() {
    pistons.set(false);
  }

  public void open() {
    pistons.set(true);
  }

  public void toggle() {
    System.out.println("************ to " + pistons.get());
    pistons.set(!pistons.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

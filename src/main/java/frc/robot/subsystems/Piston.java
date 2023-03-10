// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Piston extends SubsystemBase {
  private Solenoid leftPiston;
  private Solenoid rightPiston;
  private static Piston instance;

  /** Creates a new Piston. */
  public Piston() {
    leftPiston = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    // piston = new Solenoid(PneumaticsModuleType.CTREPCM, 17);
  }

  public static Piston getInstance() {
    if (instance == null) {
      instance = new Piston();
    }

    return instance;
  }

  public void extendPiston() {
    leftPiston.set(true);
  }

  public void retractPiston() {
    leftPiston.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

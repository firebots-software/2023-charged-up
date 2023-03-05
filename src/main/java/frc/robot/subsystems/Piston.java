// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Piston extends SubsystemBase {
  private Solenoid pistons;
  private static Piston instance;

  /** Creates a new Piston. */
  public Piston() {
    pistons = new Solenoid(PneumaticsModuleType.REVPH, 0);
  }

  public static Piston getInstance() {
    if (instance == null) {
      instance = new Piston();
    }

    return instance;
  }

  public void extendPiston() {
    pistons.set(true);
  }

  public void retractPiston() {
    pistons.set(false);
  }

  public void togglePiston() {
    System.out.println("************ to " + pistons.get());
    pistons.set(!pistons.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

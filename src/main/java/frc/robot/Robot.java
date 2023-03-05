// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SwerveSubsystem swerveSubsystem;
  private ArmSubsystem armSubsystem;

  // private PhotonVision frontCam, backCam;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    swerveSubsystem = SwerveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    // frontCam = PhotonVision.getFrontCam();
    // backCam = PhotonVision.getBackCam();
  }
    @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    armSubsystem._frictionBreakOn();
    swerveSubsystem.zeroHeading();
  }

  @Override
  public void disabledPeriodic() {
    //swerveSubsystem.zeroHeading();//
  }

  @Override
  public void disabledExit() {
    swerveSubsystem.zeroHeading();//
  }

  @Override
  public void autonomousInit() {
    swerveSubsystem.zeroHeading();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("poseX", swerveSubsystem.getPose().getX());
    SmartDashboard.putNumber("poseY", swerveSubsystem.getPose().getY());;
    
  }

  @Override
  public void autonomousExit() {
    swerveSubsystem.zeroHeading();//
  }

  @Override
  public void teleopInit() {
    swerveSubsystem.zeroHeading();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putNumber("potentiometer", clawAndArm.getPot());
  }

  @Override
  public void teleopExit() {
    swerveSubsystem.zeroHeading();//
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  
}
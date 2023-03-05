// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SwerveSubsystem swerveSubsystem;
  private ArmSubsystem armSubsystem;
  private ClawSubsystem clawSubsystem;

  private PhotonVision frontCam, backCam;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    swerveSubsystem = SwerveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    frontCam = PhotonVision.getFrontCam();
    backCam = PhotonVision.getBackCam();
    clawSubsystem = ClawSubsystem.getInstance();
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
    armSubsystem._frictionBreakOn();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    onInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("poseX", swerveSubsystem.getPose().getX());
    SmartDashboard.putNumber("poseY", swerveSubsystem.getPose().getY());
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    onInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public void onInit() {
    armSubsystem._frictionBreakOn();
    clawSubsystem.open();
    swerveSubsystem.zeroHeading();
  }

}
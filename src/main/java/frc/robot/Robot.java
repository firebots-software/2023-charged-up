// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.photo.Photo;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PhotonVision;
//import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Webcam;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private PhotonVision pv;
  private PhotonVision pv2;
  //private SwerveSubsystem swerveSubsystem;

  private RobotContainer m_robotContainer;
  private Webcam webcam;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    pv = new PhotonVision("limelightCam");
    pv2 = new PhotonVision("limelightCam2");
   // swerveSubsystem = SwerveSubsystem.getInstance();
    webcam = new Webcam();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println(photonInfo.getDistance());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("forward distance", pv.getX());
    SmartDashboard.putNumber("horizontal distance", pv.getY());
    
//    SmartDashboard.putNumber("poseX", swerveSubsystem.getPose().getX());
 //   SmartDashboard.putNumber("poseY", swerveSubsystem.getPose().getY());

    SmartDashboard.putBoolean("Limelight One Has Target", pv.hasTarget(pv.getLatestPipeline()));
    SmartDashboard.putBoolean("Limelight Two Has Target", pv.hasTarget(pv2.getLatestPipeline()));

    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  
}
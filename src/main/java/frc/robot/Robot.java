// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Webcam;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SwerveSubsystem swerveSubsystem;
  private ArmSubsystem armSubsystem;
  private ClawSubsystem clawSubsystem;
  private Webcam frontWebcam, backWebcam;
  private PhotonVision frontLimelight, backLimelight;
  private LightsSubsystem lights;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    swerveSubsystem = SwerveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    frontLimelight = PhotonVision.getFrontCam();
    // backLimelight = PhotonVision.getBackCam();
    clawSubsystem = ClawSubsystem.getInstance();
    frontWebcam = Webcam.getFrontWebcam();
    backWebcam = Webcam.getBackWebcam();
    //lights = LightsSubsystem.getInstance();

        // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(7);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    onInit();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.printAuton();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    onInit();
    swerveSubsystem.zeroPitch();
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
    swerveSubsystem.setHeading(180);
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
    clawSubsystem.close();
    swerveSubsystem.zeroHeading();
  }

}
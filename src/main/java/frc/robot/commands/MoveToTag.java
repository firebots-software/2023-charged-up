// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTag extends CommandBase {
  PhotonVision frontCam = PhotonVision.getFrontCam();
  //PhotonVision backCam = PhotonVision.getBackCam();

  PhotonVision usedCam;

  SwerveSubsystem ss;
  CommandBase pp;
  private int dir = 0;

  private double xoffset;
  private double yoffset;

  public static int LEFT = 1;
  public static int RIGHT = -1;

  /** Creates a new MoveToTarget. */
  public MoveToTag(SwerveSubsystem swerveSubsystem) {
    this.ss = swerveSubsystem;
    this.xoffset = -PhotonVision.CAM_TO_FIDUCIAL_METERS;
    this.yoffset = 0;
  }

  public MoveToTag(int pos, SwerveSubsystem swerveSubsystem) {
    this(swerveSubsystem);
    this.yoffset = -0.55 * pos; //* (DriverStation.getAlliance() != Alliance.Red ? 1 : -1);
  }

  public MoveToTag(double frontOffset, double sideOffset, SwerveSubsystem swerveSubsystem) {
    this(swerveSubsystem);
    this.xoffset = frontOffset;
    this.yoffset = sideOffset; //* (DriverStation.getAlliance() != Alliance.Red ? 1 : -1);
  }

  public int currentCamDir() {
    return dir;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double frontdist = -1;
    double backdist = -1;

    if (frontCam.hasTarget(frontCam.getLatestPipeline())) frontdist = frontCam.getDistance();
    //if (backCam.hasTarget(backCam.getLatestPipeline())) backdist = backCam.getDistance();

    if (frontdist != -1 && backdist != -1) {
      // eliminate furthest cam
      frontdist = frontdist <= backdist ? frontdist : -1;
      backdist = backdist < frontdist ? backdist : -1;
    }

    if (frontdist == -1 && backdist == -1) {
      // if neither exist, don't do anything.
      dir = 0;
      usedCam = frontCam;
    } else if (backdist == -1) {
      dir = 1;
      usedCam = frontCam;
    }/* 
    else if (frontdist == -1) {
      dir = -1;
      usedCam = backCam;
    }
    */


    pp = new MoveRelativeCmd(dir*(usedCam.getX() + xoffset), dir*(usedCam.getY() + yoffset), ss);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pp != null)
      pp.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pp != null)
      pp.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pp == null ? false : pp.isFinished();
  }
}

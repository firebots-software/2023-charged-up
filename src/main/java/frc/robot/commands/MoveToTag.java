// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTag extends CommandBase {
  PhotonVision frontCam = PhotonVision.getFrontCam();
  PhotonVision backCam = PhotonVision.getBackCam();

  PhotonVision usedCam;

  SwerveSubsystem ss;
  PPSwerveControllerCommand pp;
  private int dir;

  private double xoffset;
  private double yoffset;

  /** Creates a new MoveToTarget. */
  public MoveToTag(SwerveSubsystem swerveSubsystem) {
    this.ss = SwerveSubsystem.getInstance();
    this.xoffset = -PhotonVision.CAM_TO_FIDUCIAL_METERS;
    this.yoffset = 0;
  }

  public MoveToTag(int pos, SwerveSubsystem swerveSubsystem) {
    this(swerveSubsystem);
    this.yoffset = -0.55 * pos * (DriverStation.getAlliance() != Alliance.Red ? 1 : -1);
  }

  public MoveToTag(double frontOffset, double sideOffset, SwerveSubsystem swerveSubsystem) {
    this(swerveSubsystem);
    this.xoffset = frontOffset;
    this.yoffset = sideOffset * (DriverStation.getAlliance() != Alliance.Red ? 1 : -1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (frontCam.hasTarget(frontCam.getLatestPipeline()) && backCam.hasTarget(backCam.getLatestPipeline())) {
      double dist1 = frontCam.getDistance();
      double dist2 = backCam.getDistance();

      if (dist1 > dist2) {
        usedCam = frontCam;
        dir = 1;
      } else {
        usedCam = backCam;
        dir = -1;
      }
    } else if (!frontCam.hasTarget(frontCam.getLatestPipeline()) && backCam.hasTarget(backCam.getLatestPipeline())) {
      usedCam = backCam;
      dir = -1;
    } else {
      usedCam = frontCam;
      dir = 1;
    }

    PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2, 2),
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), //TODO: if we ever get around to calculating vision, change to ss.getHeading()
        new PathPoint(new Translation2d(dir*(usedCam.getX() + xoffset), dir*(usedCam.getY() + yoffset)),
            Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
    PathPlannerState initialSample = (PathPlannerState) traj.sample(0);
    Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
        initialSample.holonomicRotation);
    ss.resetOdometry(initialPose);

    pp = new PPSwerveControllerCommand(
        traj,
        ss::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving,
            Constants.AutonConstants.kDDriving),
        new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving,
            Constants.AutonConstants.kDDriving),
        new PIDController(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning,
            Constants.AutonConstants.kDTurning),
        ss::setModuleStates,
        ss);

    pp.initialize();
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

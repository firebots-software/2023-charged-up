// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.ArrayList;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTarget extends CommandBase {
  PhotonVision cam1 = new PhotonVision("limelightCam");
  PhotonVision cam2 = new PhotonVision("limelightCam2");

  PhotonVision usedCam;
  
  SwerveSubsystem ss;
  PPSwerveControllerCommand pp;
  Field2d field;

  /** Creates a new MoveToTarget. */
  public MoveToTarget(SwerveSubsystem swerveSubsystem) {
    field = new Field2d();
    this.ss = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(cam1.hasTarget(cam1.getLatestPipeline()) && cam2.hasTarget(cam2.getLatestPipeline())){
      double dist1 = cam1.getDistance();
      double dist2 = cam2.getDistance();

      if(dist1 > dist2){
        usedCam = cam1;
      }
      else{
        usedCam = cam2;
      }
    }
    else if(!cam1.hasTarget(cam1.getLatestPipeline()) && cam2.hasTarget(cam2.getLatestPipeline())){
      usedCam = cam2;
    }
    else{
      usedCam = cam1;
    }

    ArrayList<PathPoint> points = new ArrayList<>();
    points.clear();
    points.add(new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0)));
  
    points.add(new PathPoint(new Translation2d(usedCam.getX()-0.4,usedCam.getY()), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-ss.getHeading())));
    System.out.printf("going to %f, %f\n", usedCam.getX()-0.4, usedCam.getY());

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(2, 2), points);
    PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2, 2), 
                                                          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                                                          new PathPoint(new Translation2d(usedCam.getX()-0.4,usedCam.getY()), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-ss.getHeading())));
    field.getObject("trajectory").setTrajectory(traj);
    SmartDashboard.putData(field);
    PathPlannerState initialSample = (PathPlannerState) traj.sample(0);
    Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
      initialSample.holonomicRotation);
    ss.resetOdometry(initialPose);

    pp = new PPSwerveControllerCommand(
      traj,
      ss::getPose,
      DriveConstants.kDriveKinematics,
      new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving, Constants.AutonConstants.kDDriving),
      new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving, Constants.AutonConstants.kDDriving),
      new PIDController(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning, Constants.AutonConstants.kDTurning),
      ss::setModuleStates,
      ss);

    pp.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pp != null) pp.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pp != null) pp.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pp == null ? false : pp.isFinished();
  }
}

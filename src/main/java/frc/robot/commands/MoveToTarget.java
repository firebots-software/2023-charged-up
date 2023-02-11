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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTarget extends CommandBase {
  PhotonVision pv = new PhotonVision();
  SwerveSubsystem ss;
  PPSwerveControllerCommand pp;
  
  /** Creates a new MoveToTarget. */
  public MoveToTarget(SwerveSubsystem swerveSubsystem) {
    this.ss = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArrayList<PathPoint> points = new ArrayList<>();
    points.clear();
    points.add(new PathPoint(new Translation2d(0,0), new Rotation2d(0.0)));
    points.add(new PathPoint(new Translation2d(pv.getX()-0.4,pv.getY()), new Rotation2d(0.0)));

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(1, 1), points);

    PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
    Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
      initialSample.holonomicRotation);
    ss.resetOdometry(initialPose);

    pp = new PPSwerveControllerCommand(
      trajectory,
      ss::getPose,
      DriveConstants.kDriveKinematics,
      new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
      new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
      new PIDController(Constants.PathPlannerConstants.kPTurning, Constants.PathPlannerConstants.kITurning, Constants.PathPlannerConstants.kDTurning),
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

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveRelativeCmd extends CommandBase {
    SwerveSubsystem swerve;
    double x, y, rot;
    Command pp;

    public MoveRelativeCmd(double forwardTranslation, double leftTranslation, SwerveSubsystem swerve) {
        this(forwardTranslation, leftTranslation, 0, swerve);
    }

    public MoveRelativeCmd(double forwardTranslation, double leftTranslation, double rotationDegrees, SwerveSubsystem swerve) {
        this.x = forwardTranslation;
        this.y = leftTranslation;
        this.rot = rotationDegrees;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2, 2),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // TODO:
                new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(rot)));
        PathPlannerState initialSample = (PathPlannerState) traj.sample(0);
        Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
                initialSample.holonomicRotation);
        swerve.resetOdometry(initialPose);

        pp = new PPSwerveControllerCommand(
                traj,
                swerve::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving,
                        Constants.AutonConstants.kDDriving),
                new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving,
                        Constants.AutonConstants.kDDriving),
                new PIDController(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning,
                        Constants.AutonConstants.kDTurning),
                swerve::setModuleStates,
                true,
                swerve);

        pp.initialize();
    }

    @Override
    public void execute(){
        if (pp != null)
        pp.execute();
    }

    @Override
    public void end(boolean interrupted){
        if (pp != null)  pp.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return pp == null ? false : pp.isFinished();
    }
}

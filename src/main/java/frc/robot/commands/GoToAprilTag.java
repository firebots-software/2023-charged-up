package frc.robot.commands;

import java.util.ArrayList;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class GoToAprilTag extends CommandBase{
    private PhotonInfo pInfo;
    private SwerveSubsystem swerveSubsystem;
    private PPSwerveControllerCommand ppcontroller;
    public GoToAprilTag(PhotonInfo pInfo, SwerveSubsystem swerveSubsystem){
        this.pInfo = pInfo;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        ArrayList<PathPoint> points = new ArrayList<>();
        points.add(new PathPoint(new Translation2d(0,0), new Rotation2d(0.0)));
        points.add(new PathPoint(new Translation2d(pInfo.getY(), pInfo.getX()), new Rotation2d(0.0)));
        final PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(1, 3), points);
        PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
        Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
            initialSample.holonomicRotation);
        swerveSubsystem.resetOdometry(initialPose);
        System.out.println(trajectory);

        ppcontroller = new PPSwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
            new PIDController(Constants.PathPlannerConstants.kPDriving, Constants.PathPlannerConstants.kIDriving, Constants.PathPlannerConstants.kDDriving),
            new PIDController(Constants.PathPlannerConstants.kPTurning, Constants.PathPlannerConstants.kITurning, Constants.PathPlannerConstants.kDTurning),
            swerveSubsystem::setModuleStates,
            true,
            swerveSubsystem);
    }

    @Override
    public void execute(){
        ppcontroller.execute();
    }
    
    @Override
    public void end(boolean interrupted){
        ppcontroller.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return ppcontroller.isFinished();
    }
    
}

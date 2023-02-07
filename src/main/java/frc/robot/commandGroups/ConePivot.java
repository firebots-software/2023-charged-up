package frc.robot.commandGroups;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DockCmd;
import frc.robot.commands.EngageCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class ConePivot extends SequentialCommandGroup {
    public ConePivot(SwerveSubsystem swerveSubsystem, double armDistance, boolean pivotLeft) {

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax), 
            new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(armDistance, armDistance * (pivotLeft ? 1 : -1)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(pivotLeft ? -90 : 90))
        );

        addCommands(
        new InstantCommand(() -> {
            PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
            Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(),
                initialSample.holonomicRotation);
            swerveSubsystem.resetOdometry(initialPose);
          //the actual command that runs the path
        }),
        new PPSwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving, Constants.AutonConstants.kDDriving),
            new PIDController(Constants.AutonConstants.kPDriving, Constants.AutonConstants.kIDriving, Constants.AutonConstants.kDDriving),
            new PIDController(Constants.AutonConstants.kPTurning, Constants.AutonConstants.kITurning, Constants.AutonConstants.kDTurning),
            swerveSubsystem::setModuleStates,
            true,
            swerveSubsystem)
        );
    }
}

package frc.robot.commands;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class ConePivotCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    private double armDistance;
    boolean pivotLeft;
    
    public ConePivotCmd (SwerveSubsystem swerveSubsystem, double armDistance, boolean pivotLeft) {
        this.swerveSubsystem = swerveSubsystem;
        this.armDistance = armDistance;
        this.pivotLeft = pivotLeft;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax), 
            new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(armDistance, armDistance * (pivotLeft ? 1 : -1)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(pivotLeft ? -90 : 90))
        );
    }

    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}



package frc.robot.auto.autoModes;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.auto.TrajectoryCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commandGroups.MoveAndScore;
import frc.robot.commandGroups.PickupFromGround;
import frc.robot.commands.ToggleClaw;

public class Top2Mode extends AutoModeBase {

    private final String fileName = "Top 2";

    private final SwerveSubsystem m_swerve = SwerveSubsystem.getInstance();
    private final PhotonVision m_photonVision = PhotonVision.getInstance();
    private final ArmSubsystem m_arm = ArmSubsystem.getInstance();
    private final ClawSubsystem m_claw = ClawSubsystem.getInstance();

    private PathPlannerTrajectory traj_driveToFirstCargo;
    private PathPlannerTrajectory traj_driveToGrid;

    private TrajectoryCommand driveToFirstCargo;
    private TrajectoryCommand driveToGrid;

    public Top2Mode() {

        setup();

        addCommands(
                setup(),

                new InstantCommand(() -> {
                    m_claw.close();
                }, m_claw),

                new MoveAndScore(0, 2, m_swerve, m_arm, m_claw, true),

                new ParallelCommandGroup(
                        driveToFirstCargo,
                        new PickupFromGround(() -> true, m_arm, m_claw)),

                new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new ToggleClaw(m_claw)),

                new SequentialCommandGroup(
                        driveToGrid,
                        new MoveAndScore(0, 1, m_swerve, m_arm, m_claw, true)),
                        
                stop()
                );
                
                

    }

    public InstantCommand setup() {
        return new InstantCommand(() -> {

                SmartDashboard.putBoolean("Auto mode '" + getFileName() + "' finished", false);

                setName(fileName);

                List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                        fileName, Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

                if (pathGroup != null) {

                traj_driveToFirstCargo = pathGroup.get(0);
                traj_driveToGrid = pathGroup.get(1);

                driveToFirstCargo = new TrajectoryCommand(
                        traj_driveToFirstCargo,
                        m_swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
                        new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
                        new ProfiledPIDController(AutonConstants.kPTurning, AutonConstants.kITurning,
                                AutonConstants.kDTurning, AutonConstants.kThetaConstraints),
                        m_swerve::setModuleStates,
                        () -> false,
                        () -> false,
                        0.4,
                        m_swerve,
                        m_photonVision);

                driveToGrid = new TrajectoryCommand(
                        traj_driveToGrid,
                        m_swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
                        new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
                        new ProfiledPIDController(AutonConstants.kPTurning, AutonConstants.kITurning,
                                AutonConstants.kDTurning, AutonConstants.kThetaConstraints),
                        m_swerve::setModuleStates,
                        () -> false,
                        () -> true,
                        0.4,
                        m_swerve,
                        m_photonVision);

                } else {
                System.out.println("Error loading path group of '" + fileName + "'. Check that the file name is spelled currently and does not include '.path'");
                }
        });
    }

    public InstantCommand stop(){
        return new InstantCommand(() -> {
                SmartDashboard.putBoolean("Auto mode '" + getFileName() + "' finished", true);
        });
    }

    public Pose2d getInitialPose() {
        return traj_driveToFirstCargo.getInitialPose();
    }

    public String getFileName() {
        return fileName;
    }



}

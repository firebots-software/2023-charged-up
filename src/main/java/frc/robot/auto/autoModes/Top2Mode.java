package frc.robot.auto.autoModes;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.auto.TrajectoryCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.auto.autoModes.AutoModeBase;
import frc.robot.commandGroups.MoveAndScore;
import frc.robot.commandGroups.PickupFromGround;

public class Top2Mode extends SequentialCommandGroup implements AutoModeBase{

    private final String fileName = "Top 2";
   

    private final SwerveSubsystem m_swerve = SwerveSubsystem.getInstance();
    private final PhotonVision m_photonVision = PhotonVision.getInstance();
    private final ArmSubsystem m_arm = ArmSubsystem.getInstance();
    private final ClawSubsystem m_claw = ClawSubsystem.getInstance();
    
    

    private PathPlannerTrajectory traj_path_a;
    private PathPlannerTrajectory traj_path_b;

    private TrajectoryCommand driveToFirstCargo;
    private TrajectoryCommand driveToGrid;

    public Top2Mode(){

        setup();

        addCommands(
            new MoveAndScore(0, 2, m_swerve, m_arm, m_claw, true),
            
            new ParallelCommandGroup(
                driveToFirstCargo,
                new PickupFromGround(() -> true, m_arm, m_claw)
            ),

            new SequentialCommandGroup(
                driveToGrid,
                new MoveAndScore(0, 1, m_swerve, m_arm, m_claw, true)
            )
        );

    }

    public void setup(){

        setName(fileName);

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            fileName, Constants.AutonConstants.kVMax, Constants. AutonConstants.kAMax);

        traj_path_a = pathGroup.get(0);
        traj_path_b = pathGroup.get(1);

        driveToFirstCargo = new TrajectoryCommand(
            traj_path_a,
            m_swerve::getPose,
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
            new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
            new ProfiledPIDController(AutonConstants.kPTurning, AutonConstants.kITurning, AutonConstants.kDTurning, AutonConstants.kThetaConstraints),
            m_swerve::setModuleStates,
            () -> false,
            () -> false,
            0.4,
            m_swerve,
            m_photonVision
        );
        
        driveToGrid = new TrajectoryCommand(
            traj_path_b,
            m_swerve::getPose,
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
            new PIDController(AutonConstants.kPDriving, AutonConstants.kIDriving, AutonConstants.kDDriving),
            new ProfiledPIDController(AutonConstants.kPTurning, AutonConstants.kITurning, AutonConstants.kDTurning, AutonConstants.kThetaConstraints),
            m_swerve::setModuleStates,
            () -> false,
            () -> true,
            0.4,
            m_swerve,
            m_photonVision
        );  
    }

    public Pose2d getInitialPose(){
        return traj_path_a.getInitialPose();
    }

    public String getFileName(){
        return fileName;
    }



}

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutonConstants;

public class ResetAutoBuilder extends SwerveAutoBuilder {

  public ResetAutoBuilder(Supplier<Pose2d> poseSupplier, Consumer<Pose2d> resetPose, SwerveDriveKinematics kinematics,
      PIDConstants translationConstants, PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates, Map<String, Command> eventMap, boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, kinematics, translationConstants, rotationConstants, outputModuleStates, eventMap,
        useAllianceColor, driveRequirements);
  }

  public static List<PathPlannerTrajectory> complexTopAuton = PathPlanner.loadPathGroup("complexTopAutonPart1", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> complexBottomAuton = PathPlanner.loadPathGroup("complexBottomAutonPart2", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> topAuton = PathPlanner.loadPathGroup("topAuton", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> midAuton = PathPlanner.loadPathGroup("midAuton", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> bottomAuton = PathPlanner.loadPathGroup("bottomAuton", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> topAutonNoCharge = PathPlanner.loadPathGroup("topAutonNoCharge", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> bottomAutonNoCharge = PathPlanner.loadPathGroup("bottomAutonNoCharge", AutonConstants.kVMax, AutonConstants.kVMax);

  @Override
  public CommandBase fullAuto(List<PathPlannerTrajectory> pathGroup) {
    List<CommandBase> commands = new ArrayList<>();

    for (PathPlannerTrajectory traj : pathGroup) {
      commands.add(stopEventGroup(traj.getStartStopEvent()));
      commands.add(resetPose(traj));
      commands.add(followPathWithEvents(traj));
    }

    commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));

    return Commands.sequence(commands.toArray(CommandBase[]::new));
  }
}
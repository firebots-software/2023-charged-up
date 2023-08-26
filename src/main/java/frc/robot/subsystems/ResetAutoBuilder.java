package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;

public class ResetAutoBuilder extends SwerveAutoBuilder {

  public ResetAutoBuilder(Supplier<Pose2d> poseSupplier, Consumer<Pose2d> resetPose, SwerveDriveKinematics kinematics,
      PIDConstants translationConstants, PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates, Map<String, Command> eventMap, boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, kinematics, translationConstants, rotationConstants, outputModuleStates, eventMap,
        useAllianceColor, driveRequirements);
  }

  public static enum ChargeStationOptions {
    NO_CHARGE,
    CHARGE
  }

  public static enum PiecesScoredOptions {
    ONE,
    TWO
  }

  public static enum StartingPositionOptions {
    JUST_SCORE,
    TOP,
    MIDDLE,
    BOTTOM
  }

  public static int hashAuton(StartingPositionOptions startPos, PiecesScoredOptions pieces, ChargeStationOptions charge) {
    return (startPos.ordinal() + (pieces.ordinal() << 2) + (charge.ordinal() << 3)) * (startPos == StartingPositionOptions.JUST_SCORE ? 0 : 1);
  }

  public Command autoFromPath(String name, double velOverride) {
    try {
      List<PathPlannerTrajectory> ppts = PathPlanner.loadPathGroup(name, velOverride, AutonConstants.kAMax);
      if (ppts == null) {
        return new WaitCommand(1);
      }
      Command n = fullAuto(ppts);
      n.setName(name);
      return n;
    } catch (Exception e) {
      return new WaitCommand(1);
    }
  }

  public Command autoFromPath(String name) {
    return autoFromPath(name, AutonConstants.kVMax);
  }

  

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

  public static List<PathPlannerTrajectory>  Top_Charge_2 = PathPlanner.loadPathGroup(
    "Top Charge 2", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> Top_Charge_1 = PathPlanner.loadPathGroup(
    "Top Charge 1", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> Mid_Charge_1 = PathPlanner.loadPathGroup(
    "Mid Charge 1", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> Bottom_Charge_1 = PathPlanner.loadPathGroup(
    "Bottom Charge 1", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> Top_1 = PathPlanner.loadPathGroup(
    "Top 1", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> Bottom_1 = PathPlanner.loadPathGroup(
    "Bottom 1", AutonConstants.kVMax, AutonConstants.kVMax);

  public static List<PathPlannerTrajectory> Top_2 = PathPlanner.loadPathGroup(
    "Top 2", AutonConstants.kVMax, AutonConstants.kAMax);
}
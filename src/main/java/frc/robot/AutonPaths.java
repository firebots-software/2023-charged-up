package frc.robot;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonPaths {

    public static List<PathPlannerTrajectory> oranjeup = PathPlanner.loadPathGroup(
            "oranjeUp",
            Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    public static List<PathPlannerTrajectory> oranjemid = PathPlanner.loadPathGroup(
            "oranjeMid",
            Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    public static List<PathPlannerTrajectory> oranjeComplexTop = PathPlanner.loadPathGroup(
            "oranjeComplexTop",
            Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    public static List<PathPlannerTrajectory> oranjeComplexBottom = PathPlanner.loadPathGroup(
            "oranjeComplexBottom",
            Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    public static List<PathPlannerTrajectory> oranjeComplexMid = PathPlanner.loadPathGroup(
            "oranjeComplexMid",
            Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);
}

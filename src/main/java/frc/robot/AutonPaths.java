package frc.robot;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonPaths {


    public static List<PathPlannerTrajectory> topAuton = PathPlanner.loadPathGroup(
        "topAuton", 
        Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);
    
    public static List<PathPlannerTrajectory> bottomAuton = PathPlanner.loadPathGroup(
        "bottomAuton", 
        Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    public static List<PathPlannerTrajectory> middleAuton = PathPlanner.loadPathGroup(
        "middleAuton", 
        Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    public static List<PathPlannerTrajectory> testTopAutonPart1 = PathPlanner.loadPathGroup(
        "testTopAutonPart1", 
        Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);
        
    public static List<PathPlannerTrajectory> testTopAutonPart2 = PathPlanner.loadPathGroup(
        "testTopAutonPart2", 
        Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

        public static List<PathPlannerTrajectory> chargeStationAndMobility = PathPlanner.loadPathGroup(
            "chargeStation", 
            Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);
}

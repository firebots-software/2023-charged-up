package frc.robot;

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

        
    
}

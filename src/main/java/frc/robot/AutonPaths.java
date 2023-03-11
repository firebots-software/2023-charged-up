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
}

package frc.robot;


import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.AutonConstants;

public class AutonPaths {

    public static List<List<PathPlannerTrajectory>> complexTopAuton = makePPTList(
        "complexTopAutonPart1", "complexTopAutonPart2", "complexTopAutonPart3"
    );

    public static List<List<PathPlannerTrajectory>> complexBottomAuton = makePPTList(
        "complexBottomAutonPart1", "complexBottomAutonPart2", "complexBottomAutonPart3"
    );

    public static List<List<PathPlannerTrajectory>> topAuton = makePPTList(
        "topAuton"
    );

    public static List<List<PathPlannerTrajectory>> midAuton = makePPTList(
        "midAuton"
    );

    public static List<List<PathPlannerTrajectory>> bottomAuton = makePPTList(
        "bottomAuton"
    );

    public static List<List<PathPlannerTrajectory>> topAutonNoCharge = makePPTList(
        "topAutonNoCharge"
    );
    
    public static List<List<PathPlannerTrajectory>> bottomAutonNoCharge = makePPTList(
        "bottomAutonNoCharge"
    );

    


    private static List<List<PathPlannerTrajectory>> makePPTList(String... pathNames) {
        List<List<PathPlannerTrajectory>> ppts = new ArrayList<List<PathPlannerTrajectory>>();
        for (String pathName : pathNames){
            ppts.add(PathPlanner.loadPathGroup(pathName, AutonConstants.kVMax, AutonConstants.kVMax));
        }
        return ppts;
    }
    
}

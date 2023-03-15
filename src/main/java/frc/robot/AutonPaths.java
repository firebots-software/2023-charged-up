package frc.robot;


import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.AutonConstants;

public class AutonPaths {

    public static List<List<PathPlannerTrajectory>> testTopAuton = makePPTList(
        "testTopAutonPart1", "testTopAutonPart2", "testTopAutonPart3"
    );

    // public static List<PathPlannerTrajectory> topAuton = makePPT(
    //     "topAuton"
    //     );
    
    // public static List<PathPlannerTrajectory> bottomAuton = makePPT(
    //     "bottomAuton"
    //     );

    // public static List<PathPlannerTrajectory> middleAuton = makePPT(
    //     "middleAuton"
    //     );
    
    // public static List<PathPlannerTrajectory> chargeStationAndMobility = makePPT(
    //     "chargeStation"
    //     );


    private static List<List<PathPlannerTrajectory>> makePPTList(String... pathNames) {
        List<List<PathPlannerTrajectory>> ppts = new ArrayList<List<PathPlannerTrajectory>>();
        for (String pathName : pathNames){
            ppts.add(PathPlanner.loadPathGroup(pathName, AutonConstants.kVMax, AutonConstants.kVMax));
        }
        return ppts;
    }
    
    // public static List<PathPlannerTrajectory> oranjeup = PathPlanner.loadPathGroup(
    //         "oranjeTopNoCharge",
    //         Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    // public static List<PathPlannerTrajectory> oranjebottom = PathPlanner.loadPathGroup(
    //         "oranjeBottomNoCharge",
    //         Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    // public static List<PathPlannerTrajectory> oranjeComplexTop = PathPlanner.loadPathGroup(
    //         "oranjeComplexTop",
    //         Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    // public static List<PathPlannerTrajectory> oranjeComplexBottom = PathPlanner.loadPathGroup(
    //         "oranjeComplexBottom",
    //         Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);

    // public static List<PathPlannerTrajectory> oranjeComplexMid = PathPlanner.loadPathGroup(
    //         "oranjeComplexMid",
    //         Constants.AutonConstants.kVMax, Constants.AutonConstants.kAMax);
}

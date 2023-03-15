package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.FormatFlagsConversionMismatchException;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.AutonConstants;

public class AutonPaths {

    public static List<List<PathPlannerTrajectory>> testTopAuton = makePPTList(
        "testTopAutonPart1", "testTopAutonPart2"
    );

    public static List<List<PathPlannerTrajectory>> topAuton = makePPTList(
        "topAuton"
        );
    
    public static List<List<PathPlannerTrajectory>> bottomAuton = makePPTList(
        "bottomAuton"
        );

    public static List<List<PathPlannerTrajectory>> middleAuton = makePPTList(
        "middleAuton"
        );
    
    public static List<List<PathPlannerTrajectory>> chargeStationAndMobility = makePPTList(
        "chargeStation"
        );


    /**
     * 
     * @param pathNames File name of each trajectory, with out the ".path"
     * @return List comprised of List of PathPlannerTrajectories
     */
    private static List<List<PathPlannerTrajectory>> makePPTList(String... pathNames) {

        List<List<PathPlannerTrajectory>> ppts = new ArrayList<List<PathPlannerTrajectory>>();

        for (String pathName : pathNames) {
            ppts.add(
                PathPlanner.loadPathGroup(pathName, AutonConstants.kVMax, AutonConstants.kAMax)
            );
        }

        return ppts;
    }
    
}

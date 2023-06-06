package frc.robot.auto.autoModes;

import edu.wpi.first.math.geometry.Pose2d;

public class DoNothingMode extends AutoModeBase {

    private final String fileName = "";

    public DoNothingMode(){
        setup();
    }

    @Override
    public void setup() {
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }

    public String getFileName(){
        return fileName;
    }
    
}

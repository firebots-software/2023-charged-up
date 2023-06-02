package frc.robot.auto.autoModes;

import edu.wpi.first.math.geometry.Pose2d;

public interface AutoModeBase {
    
    public abstract void setup();

    public abstract Pose2d getInitialPose();

    public abstract String getFileName();

}

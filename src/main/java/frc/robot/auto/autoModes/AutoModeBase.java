package frc.robot.auto.autoModes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutoModeBase extends SequentialCommandGroup {
    
    public abstract void setup();

    public abstract Pose2d getInitialPose();

    public abstract String getFileName();

}

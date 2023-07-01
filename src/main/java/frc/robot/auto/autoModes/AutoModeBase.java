package frc.robot.auto.autoModes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AutoModeBase extends SequentialCommandGroup {
    
    public abstract InstantCommand setup();

    public abstract InstantCommand stop();

    public abstract Pose2d getInitialPose();

    public abstract String getFileName();

}

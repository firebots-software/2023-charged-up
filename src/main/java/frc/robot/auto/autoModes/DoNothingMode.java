package frc.robot.auto.autoModes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DoNothingMode extends SequentialCommandGroup implements AutoModeBase {
    public DoNothingMode(){
        setup();
        stop();
    }

    @Override
    public void setup() {
        SmartDashboard.putBoolean("Auto finished", true);
        System.out.println("Auto finished!");
    }

    @Override
    public void stop() {}

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}

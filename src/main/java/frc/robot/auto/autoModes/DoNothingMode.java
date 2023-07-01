package frc.robot.auto.autoModes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DoNothingMode extends AutoModeBase {

    private final String fileName = "";

    public DoNothingMode(){
        addCommands(
            setup(),
            stop()
        );
    }

    @Override
    public InstantCommand setup() {
        return new InstantCommand();
    }

    public InstantCommand stop(){
        return new InstantCommand(() -> {
                SmartDashboard.putBoolean("Auto mode '" + getFileName() + "' finished", true);
        });
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }

    public String getFileName(){
        return fileName;
    }


    
}

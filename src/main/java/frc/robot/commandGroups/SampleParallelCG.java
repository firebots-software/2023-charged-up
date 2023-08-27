package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SampleCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class SampleParallelCG extends ParallelCommandGroup {
    // notice how this extends ParallelCommandGroup
    public SampleParallelCG(SwerveSubsystem swerve) {
        addCommands(
            // add any commands you want to run at the same time here
            new SampleCmd(swerve),
            new SampleCmd(swerve),

            // you can even nest other command groups! 
            // the bottom two commands will run one after the other, even while the first two are still running
            new SequentialCommandGroup(
                new SampleCmd(swerve),
                new SampleCmd(swerve)
            )
        );
        
        // you don't need to addRequirements() here: the commands themseleves take care of that.
    }
}

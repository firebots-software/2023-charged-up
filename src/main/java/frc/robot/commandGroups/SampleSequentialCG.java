package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SampleCmd;

public class SampleSequentialCG extends SequentialCommandGroup {
    // notice how this extends SequentialCommandGroup
    public SampleSequentialCG() {
        addCommands(
            // add any commands you want to run one after the other here
            // once the first command finished, the second will run.
            new SampleCmd(),
            new SampleCmd(),

            // you can even nest other command groups! 
            // the bottom two commands will run at the same time after the above two commands finish.
            new ParallelCommandGroup(
                new SampleCmd(),
                new SampleCmd()
            )
        );

        // you don't need to addRequirements() here: the commands themseleves take care of that.
    }
}

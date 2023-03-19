package frc.robot.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.JankArmToTicks;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToGround extends SequentialCommandGroup {
    public ArmToGround(Supplier<Boolean> back, ArmSubsystem arm, boolean auton) {
        addCommands(
            new ArmToDegree(arm, () -> 125d * (back.get() ? 1d : -1d)),
            new JankArmToTicks(277552, arm)
        );
    }

    public ArmToGround(Supplier<Boolean> back, ArmSubsystem arm) {
        addCommands(
            new ArmToDegree(arm, () -> 125d * (back.get() ? 1d : -1d))
            //new JankArmToTicks(277552, arm)
        );
    }
}

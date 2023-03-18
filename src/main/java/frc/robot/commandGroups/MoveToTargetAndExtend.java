package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.MoveToTag;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTargetAndExtend extends ParallelCommandGroup{
    public MoveToTargetAndExtend(SwerveSubsystem swerveSubsystem, ArmSubsystem arm){
        addCommands(
            new MoveToTag(swerveSubsystem),
            new JankArmToTicks(228818, arm)
        );
    }
}

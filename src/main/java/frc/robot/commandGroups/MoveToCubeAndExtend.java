package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.JankArmToTicks;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToCubeAndExtend extends ParallelCommandGroup{
    public MoveToCubeAndExtend(SwerveSubsystem swerveSubsystem, ArmSubsystem arm){
        addCommands(
            // new MoveToCube(swerveSubsystem),
            new JankArmToTicks(304433, arm)
        );
    }
}

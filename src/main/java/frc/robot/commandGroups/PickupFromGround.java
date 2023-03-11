package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmToDegree;
import frc.robot.commands.ExtendArmCmd;
import frc.robot.commands.JankArmToTicks;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class PickupFromGround extends SequentialCommandGroup {
    public PickupFromGround(ArmSubsystem arm, ClawSubsystem claw) {
        addCommands(
            new ToggleClaw(true, claw),
            new ArmToDegree(arm, 120),
            new JankArmToTicks(277552, arm),
            new ToggleClaw(false, claw),
            new WaitCommand(.75),
            new RetractArmCmd(arm)
        );
    }
}

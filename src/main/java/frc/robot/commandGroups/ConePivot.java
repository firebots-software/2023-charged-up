package frc.robot.commandGroups;

import frc.robot.commands.MoveRelativeCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class ConePivot extends MoveRelativeCmd {
    public ConePivot(SwerveSubsystem swerveSubsystem, double armDistance, boolean pivotLeft) {
        super(armDistance, armDistance * (pivotLeft ? 1 : -1), 0, swerveSubsystem);
    }
}

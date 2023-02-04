package frc.robot.commandGroups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DockCmd;
import frc.robot.commands.EngageCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeStation extends SequentialCommandGroup {
    public ChargeStation(SwerveSubsystem swerveSubsystem) {
        addCommands(
            new DockCmd(swerveSubsystem, Constants.DockingConstants.DOCKING_SPEED),
            new EngageCmd(new PIDController(Constants.DockingConstants.kPEngage, Constants.DockingConstants.KIEngage, Constants.DockingConstants.KDEngage), swerveSubsystem)
        );
    }
}

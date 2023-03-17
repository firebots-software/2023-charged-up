package frc.robot.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DockCmd;
import frc.robot.commands.EngageCmd;
import frc.robot.commands.JankEngageCmd;
import frc.robot.commands.LevelCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeStation extends SequentialCommandGroup {
    public ChargeStation(SwerveSubsystem swerveSubsystem, double speed) {
        addCommands(
            new DockCmd(swerveSubsystem, speed),
            new LevelCmd(swerveSubsystem, 1.0*Math.signum(speed)),
            //new EngageCmd(new PIDController(Constants.DockingConstants.kPEngage, Constants.DockingConstants.KIEngage, Constants.DockingConstants.KDEngage), swerveSubsystem)
            new JankEngageCmd(swerveSubsystem)
        );
    }
}

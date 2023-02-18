package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DockCmd extends CommandBase {

    private SwerveSubsystem swerveSubsystem;
    private final SwerveModuleState[] forwardModuleStates;

    public DockCmd(SwerveSubsystem swerveSubsystem, double speed) {
        this.swerveSubsystem = swerveSubsystem;
        SwerveModuleState moduleState = new SwerveModuleState(speed, new Rotation2d());
        this.forwardModuleStates = new SwerveModuleState[]{moduleState, moduleState, moduleState, moduleState};

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.setModuleStates(forwardModuleStates);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerveSubsystem.getPitch()) > Constants.DockingConstants.DOCKING_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}

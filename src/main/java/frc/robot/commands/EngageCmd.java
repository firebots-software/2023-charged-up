package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class EngageCmd extends CommandBase{

    private PIDController pid; 
    private SwerveSubsystem swerve;

    public EngageCmd(PIDController pid, SwerveSubsystem swerve) {
        this.pid = pid;
        this.swerve = swerve;

        pid.setSetpoint(swerve.getPitch());

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        double pitch = swerve.getPitch();
        double calculated = pid.calculate(pitch);
        SwerveModuleState state = new SwerveModuleState(calculated, new Rotation2d(0d));
        swerve.setModuleStates(new SwerveModuleState[]{state, state, state, state});
    }


    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
        // stop motors
    }

    @Override
    public boolean isFinished() {
        return false; // stop when robot is leveled, gyro gives zero
    }

}

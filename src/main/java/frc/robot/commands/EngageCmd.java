package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class EngageCmd extends CommandBase{

    private PIDController pid; 
    private SwerveSubsystem swerve;

    public EngageCmd(PIDController pid, SwerveSubsystem swerve) {
        this.pid = pid;
        this.swerve = swerve;

        pid.setSetpoint(swerve.getPitch());
        pid.setTolerance(0.5, 0.0005);

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
        double clamped = MathUtil.clamp(calculated, -1, 1);
        SwerveModuleState state = new SwerveModuleState(clamped, new Rotation2d(0d));
        swerve.setModuleStates(new SwerveModuleState[]{state, state, state, state});
        SmartDashboard.putNumber("pid calculated value", calculated);
        SmartDashboard.putNumber("clamped value", clamped);
        
        
    }


    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
        // stop motors
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint(); // stop when robot is leveled, gyro gives zero
    }

}

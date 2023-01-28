package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeStation extends CommandBase{

    private PIDController pid; 
    private SwerveSubsystem swerve;
    private double[] angles;

    public ChargeStation(PIDController pid, SwerveSubsystem swerve) {
        this.pid = pid;
        this.swerve = swerve;
        angles = new double[3];

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
        swerve.setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(calculated, new Rotation2d(0d)),
            new SwerveModuleState(calculated, new Rotation2d(0d)),
            new SwerveModuleState(calculated, new Rotation2d(0d)),
            new SwerveModuleState(calculated, new Rotation2d(0d))
        });
        // get gyro position and change PID
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

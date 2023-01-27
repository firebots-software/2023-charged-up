package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class RunForDistance extends CommandBase {
    private final double meters;
    private final SwerveSubsystem swerve;
    private Supplier<Boolean> printer;
    private double start;

    public RunForDistance(double meters, Supplier<Boolean> printer, SwerveSubsystem swerve) {
        this.meters = meters;
        this.swerve = swerve;
        this.printer = printer;

        addRequirements(swerve);
    }

    
    @Override
    public void initialize() {
        start = swerve.getModulePositions()[0].distanceMeters;
    }

    @Override
    public void execute() {
        double dist = swerve.getModulePositions()[0].distanceMeters;
        if (printer.get()) {
            SmartDashboard.putNumber("Distance traveled", dist);
            System.out.println(dist);
        }

        if (dist - start < meters) {
            swerve.setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(1, new Rotation2d(0)),
                new SwerveModuleState(1, new Rotation2d(0)),
                new SwerveModuleState(1, new Rotation2d(0)),
                new SwerveModuleState(1, new Rotation2d(0)),
            });
        } else {
            swerve.stopModules();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DockingConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class JankEngageCmd extends CommandBase{

    private SwerveSubsystem swerve;
    double startPitch = 0;
    double speed = 0.5;
    Supplier<Double> initial;

    public JankEngageCmd(SwerveSubsystem swerve, Supplier<Double> initialPitch) {
        this.swerve = swerve;
        this.initial = initialPitch;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startPitch = initial.get();
    }

    @Override
    public void execute() {
        double pitch = swerve.getPitch() - startPitch;
        
        if (pitch>DockingConstants.DOCKING_TOLERANCE) {
            SwerveModuleState state = new SwerveModuleState(-speed, new Rotation2d(0d));
            swerve.setModuleStates(new SwerveModuleState[]{state, state, state, state});
        }
        else if (pitch<-DockingConstants.DOCKING_TOLERANCE){
            SwerveModuleState state = new SwerveModuleState(speed, new Rotation2d(0d));
            swerve.setModuleStates(new SwerveModuleState[]{state, state, state, state});
        }
        else{
            swerve.stopModules();
            swerve.setTurning(Math.PI / 2.0);
        }
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

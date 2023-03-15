package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DockingConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class JankEngageCmd extends CommandBase{

    private SwerveSubsystem swerve;
    double speed = 0.42;

    public JankEngageCmd(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double pitch = swerve.getPitch();
        
        if (pitch>DockingConstants.ENGAGE_TOLERANCE) {
            SwerveModuleState state = new SwerveModuleState(-speed, new Rotation2d(0d));
            swerve.setModuleStates(new SwerveModuleState[]{state, state, state, state});
        }
        else if (pitch<-DockingConstants.ENGAGE_TOLERANCE){
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

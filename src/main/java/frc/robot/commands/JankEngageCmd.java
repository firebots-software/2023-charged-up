package frc.robot.commands;

import java.util.function.Supplier;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DockingConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class JankEngageCmd extends CommandBase {

    private SwerveSubsystem swerve;
    private double startPitch;
    private double speed;

    public JankEngageCmd(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.speed = DockingConstants.ENGAGE_SPEED;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startPitch = swerve.getPitch();
    }

    @Override
    public void execute() {
        double pitch = swerve.getPitch();

        if (pitch > DockingConstants.ENGAGE_TOLERANCE) {
            SwerveModuleState state = new SwerveModuleState(-speed, new Rotation2d(0d));
            swerve.setModuleStates(new SwerveModuleState[] { state, state, state, state });
        } else if (pitch < -DockingConstants.ENGAGE_TOLERANCE) {
            SwerveModuleState state = new SwerveModuleState(speed, new Rotation2d(0d));
            swerve.setModuleStates(new SwerveModuleState[] { state, state, state, state });
        } else {
            swerve.stopModules();
            swerve.setTurning(Math.PI / 2.0);
        }

        if (Math.abs(pitch - startPitch) > 2 * DockingConstants.ENGAGE_TOLERANCE - 2.0) {
            this.startPitch = pitch;
            this.speed -= DockingConstants.OSCILLATION_SPEED_INCREMENT;
            
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

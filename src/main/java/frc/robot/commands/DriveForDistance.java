package frc.robot.commands;

import java.util.function.Supplier;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OI;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForDistance extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final PIDController pid;


    public DriveForDistance(){
        swerveSubsystem = new SwerveSubsystem();
        pid = new PIDController(1, 0, 0);
    }
    

    @Override
    public void initialize() {
        pid.setSetpoint(43140.35163);
  
        System.out.println("*****************************\n*****************************\n*****************************\n*****************************\n");
    }

    @Override
    public void execute() {
        double pidOutput = pid.calculate(swerveSubsystem.getPosition());
        System.out.println(pidOutput);
        swerveSubsystem.setFrontLeft(pidOutput); 
        swerveSubsystem.setFrontRight(pidOutput);
        swerveSubsystem.setBackLeft(pidOutput); 
        swerveSubsystem.setBackRight(pidOutput); 
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
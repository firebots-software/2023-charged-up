package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClawAndArm;
import frc.robot.Constants.clawConstants;



public class RotateArmToAngle extends CommandBase{
    protected ClawAndArm clawAndArm; 
    private PIDController pid;

    public RotateArmToAngle(double targetAngle) {
        clawAndArm = ClawAndArm.getInstance(); 
        pid = new PIDController(clawConstants.angleP, clawConstants.angleI, clawConstants.angleD);
        pid.setTolerance(clawConstants.pidPositionToleranceDegrees);
        pid.setSetpoint(targetAngle);
    }

    @Override
    public void initialize() {
        
    }  

    @Override
    public void execute() {
        double angle = clawAndArm.getAngle();
        double val = pid.calculate(angle);
        
        clawAndArm.setRotatingMotor(val);
    }

    @Override
    public void end(boolean interrupted) {
        clawAndArm.setRotatingMotor(0);
    }

    public boolean isFinished() {
        return pid.atSetpoint();
      }

      @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(clawAndArm);
    }
}



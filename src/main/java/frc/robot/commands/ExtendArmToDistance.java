package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.clawConstants;
import frc.robot.subsystems.ClawAndArm;

public class ExtendArmToDistance extends CommandBase{
    protected ClawAndArm clawAndArm;

    private double targetHeight;
    private PIDController pid;

    public ExtendArmToDistance(double targetHeight) {
        this.clawAndArm = ClawAndArm.getInstance();
        pid = new PIDController(clawConstants.distP, clawConstants.distI, clawConstants.distD);
        pid.setTolerance(clawConstants.pidPositionToleranceInches);
      }

    public void initialize() {
        pid.setSetpoint(targetHeight);
    }

    @Override
    public void execute() {
        double position = clawAndArm.getPosition();
        double val = pid.calculate(position);
        
        clawAndArm.setExtendingMotor(val);
    }

    @Override
    public void end(boolean interrupted) {
        clawAndArm.setExtendingMotor(0);
    }

    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(clawAndArm);
    }
 
     
}
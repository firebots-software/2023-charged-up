package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.clawConstants;
import frc.robot.subsystems.ClawAndArm;

public class ExtendArmToDistance extends CommandBase{
    protected ClawAndArm clawAndArm;
    private PIDController pid;

    public ExtendArmToDistance(double targetDistance) {
        this.clawAndArm = ClawAndArm.getInstance();
        pid = new PIDController(clawConstants.distP, clawConstants.distI, clawConstants.distD);
        pid.setTolerance(clawConstants.pidPositionToleranceInches);
        pid.setSetpoint(targetDistance);
      }

    public void initialize() {
        
    }

    // @Override
    // public void execute() {
    //     System.out.println("doing command");
    //     double position = clawAndArm.getPosition();
    //     System.out.println(position);
    //     double val = pid.calculate(position);
    //     System.out.println(val);
        
    //     clawAndArm.setExtendingMotor(val);
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     clawAndArm.setExtendingMotor(0);
    // }

    // public boolean isFinished() {
    //     return pid.atSetpoint();
    // }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(clawAndArm);
    }
 
     
}
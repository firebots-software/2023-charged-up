package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmToDistance extends CommandBase{
    protected ArmSubsystem clawAndArm;
    private PIDController pid;

    public ExtendArmToDistance(double targetDistance) {
        this.clawAndArm = ArmSubsystem.getInstance();
        pid = new PIDController(ArmConstants.distP, ArmConstants.distI, ArmConstants.distD);
        pid.setTolerance(ArmConstants.pidPositionToleranceInches);
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
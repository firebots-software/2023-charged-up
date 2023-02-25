package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;



public class RotateArmToAngle extends CommandBase{
    protected ArmSubsystem clawAndArm; 
    private PIDController pid;

    public RotateArmToAngle(double targetAngle) {
        this.clawAndArm = ArmSubsystem.getInstance(); 
        pid = new PIDController(ArmConstants.angleP, ArmConstants.angleI, ArmConstants.angleD);
        pid.setTolerance(ArmConstants.pidPositionToleranceDegrees);
        pid.setSetpoint(targetAngle);
    }

    @Override
    public void initialize() {
        
    }  

    @Override
    public void execute() {
        double angle = clawAndArm._getPotentiometerDegrees();
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



package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClawAndArm;
import frc.robot.Constants.clawConstants;



public class ClawToAngle extends CommandBase{
    private ClawAndArm claw; 
    private PIDController pid;

    public static final double LOWEST_POS = 130;

    public ClawToAngle() {
        claw = ClawAndArm.getInstance(); 
        pid = new PIDController(clawConstants.angleP, clawConstants.angleI, clawConstants.angleD);
        pid.setTolerance(clawConstants.pidPositionToleranceDegrees);

        addRequirements(claw);

    }

    @Override
    public void initialize() {
        pid.setSetpoint(clawConstants.TARGET_ANGLE);
    }  

    @Override
    public void execute() {
        double angle = claw.getDegrees();
        double val = pid.calculate(angle);
        
        claw.setRotatingMotor(val);
    }

    @Override
    public void end(boolean interrupted) {
        claw.setRotatingMotor(0);
        System.out.println("done with command");
    }

    public boolean isFinished() {
        return pid.atSetpoint();
      }
}



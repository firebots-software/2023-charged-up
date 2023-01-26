package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.clawConstants;



public class ClawToPosition extends CommandBase{
    private Claw claw; 
    private PIDController pid;

    public ClawToPosition() {
        claw = Claw.getInstance(); 
    }

    @Override
    public void initialize() {
        pid.setSetpoint(clawConstants.TARGET_ANGLE);
    }  

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return pid.atSetpoint();
      }

      @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(claw);
    }
}



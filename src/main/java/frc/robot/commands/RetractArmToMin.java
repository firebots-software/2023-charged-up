package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClawAndArm;

public class RetractArmToMin extends CommandBase{
    protected ClawAndArm clawAndArm;

    private double motorVoltage;

    public RetractArmToMin(double motorVoltage) {
        this.clawAndArm = ClawAndArm.getInstance();
        this.motorVoltage = motorVoltage;
      }

    public void initialize() {}

    @Override
    public void execute() {
if (clawAndArm.getBottomStatus() != true) {
    clawAndArm.setExtendingMotor(-motorVoltage);
}
    }

    @Override
    public void end(boolean interrupted) {
        clawAndArm.setExtendingMotor(0);
    }

    public boolean isFinished() {
        return clawAndArm.getBottomStatus();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(clawAndArm);
    }
 
     
}
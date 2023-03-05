package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmToMax extends CommandBase{
    protected ArmSubsystem arm;

    private double motorVoltage;

    public ExtendArmToMax(double motorVoltage) {
        this.arm = ArmSubsystem.getInstance();
        this.motorVoltage = motorVoltage;
      }

    public void initialize() {}

    @Override
    public void execute() {
if (!arm.getTopStatus()) {
    arm.setExtendingMotor(motorVoltage);
}
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtendingMotor(0);
    }

    public boolean isFinished() {
        return arm.getTopStatus();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(arm);
    }
 
     
}
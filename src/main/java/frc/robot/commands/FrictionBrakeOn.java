package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class FrictionBrakeOn extends CommandBase {
    protected ArmSubsystem ClawAndArm;

    public FrictionBrakeOn() {
        ClawAndArm = ClawAndArm.getInstance();
    }

    @Override
    public void initialize() {
        ClawAndArm._frictionBreakOn();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        ClawAndArm._frictionBreakOff();
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(ClawAndArm);
    }
}

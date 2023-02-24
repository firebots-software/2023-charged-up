package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClawAndArm;
import frc.robot.subsystems.ClawAndArm;

public class FrictionBreakOn extends CommandBase {
    protected ClawAndArm ClawAndArm;

    public FrictionBreakOn() {
        ClawAndArm = ClawAndArm.getInstance();
    }

    @Override
    public void initialize() {
        ClawAndArm.frictionBreakOn();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        ClawAndArm.frictionBreakOff();
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(ClawAndArm);
    }
}

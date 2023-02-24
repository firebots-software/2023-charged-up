package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawAndArm;

public class ZeroArmCmd extends CommandBase {
    private final ClawAndArm claw = ClawAndArm.getInstance();
    
    public ZeroArmCmd() {
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.resetRotation();
    }

    @Override
    public void execute() {
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

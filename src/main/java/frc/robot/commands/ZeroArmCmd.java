package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroArmCmd extends CommandBase {
    private final ArmSubsystem claw = ArmSubsystem.getInstance();
    
    public ZeroArmCmd() {
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setRotation(0);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}

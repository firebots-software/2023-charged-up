package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmCmd extends CommandBase {
    private ArmSubsystem arm;

    public ExtendArmCmd(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setExtendingMotor(.3);
    }

    @Override
    public boolean isFinished() {
        return arm.getTopStatus();
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtendingMotor(0);
    }
}

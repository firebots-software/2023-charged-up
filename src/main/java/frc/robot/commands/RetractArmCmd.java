package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RetractArmCmd extends CommandBase {
    private ArmSubsystem arm;

    public RetractArmCmd(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setExtendingMotor(-.4);
    }

    @Override
    public boolean isFinished() {
        return arm.getBottomStatus();
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtendingMotor(0);
    }
}

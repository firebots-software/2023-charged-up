package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class JankArmToTicks extends CommandBase {
    private ArmSubsystem arm;
    private double ticks;

    public JankArmToTicks(double ticks, ArmSubsystem arm) {
        this.ticks = ticks;
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setExtendingMotor(0.35 * Math.signum(ticks-arm.getTicks())); 
        System.out.println("************* EXTENDING *************");// retract arm
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtendingMotor(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(ticks - arm.getTicks()) < 1000; // if within a thousand ticks, it's probably good enough?
    }
}

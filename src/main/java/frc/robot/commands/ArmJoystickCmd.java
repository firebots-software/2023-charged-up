package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OI;
import frc.robot.subsystems.ClawAndArm;

public class ArmJoystickCmd extends CommandBase {
    private ClawAndArm claw = ClawAndArm.getInstance();
    private final Supplier<Double> armspdfunc;

    public ArmJoystickCmd(Supplier<Double> rotation) {
        this.armspdfunc = rotation;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = armspdfunc.get();
        if (Math.abs(speed) > OI.DEADBAND) {
            claw.setRotatingMotor(0);
            claw.frictionBreakOn();
            return;
        }
        claw.frictionBreakOff();

        claw.setRotatingMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        claw.setRotatingMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

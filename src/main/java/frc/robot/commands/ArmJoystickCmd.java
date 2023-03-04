package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OI;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCmd extends CommandBase {
    private ArmSubsystem claw = ArmSubsystem.getInstance();
    private final Supplier<Double> armspdfunc, extspdfunc;

    public ArmJoystickCmd(Supplier<Double> rotation, Supplier<Double> extension) {
        this.armspdfunc = rotation;
        this.extspdfunc = extension;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double angleSpeed = armspdfunc.get();
        double extSpeed = extspdfunc.get();

        angleSpeed = Math.abs(angleSpeed) > OI.DEADBAND ? angleSpeed : 0.0;
        extSpeed = Math.abs(extSpeed) > OI.DEADBAND ? extSpeed : 0.0;

        claw.setRotatingMotor(angleSpeed);
        claw.setExtendingMotor(extSpeed);
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

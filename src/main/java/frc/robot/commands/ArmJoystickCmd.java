package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OI;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCmd extends CommandBase {
    private ArmSubsystem arm;
    private final Supplier<Double> armspdfunc, extspdfunc;

    public ArmJoystickCmd(ArmSubsystem arm, Supplier<Double> rotation, Supplier<Double> extension) {
        this.arm = arm;
        this.armspdfunc = rotation;
        this.extspdfunc = extension;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double angleSpeed = armspdfunc.get();
        double extSpeed = extspdfunc.get();

        angleSpeed = Math.abs(angleSpeed) > OI.ARM_DEADBAND ? angleSpeed : 0.0;
        extSpeed = Math.abs(extSpeed) > OI.ARM_DEADBAND ? extSpeed : 0.0;

        arm.setRotatingMotor(angleSpeed);
        arm.setExtendingMotor(extSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setRotatingMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

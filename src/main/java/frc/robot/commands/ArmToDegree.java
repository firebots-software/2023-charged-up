package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ArmToDegree extends CommandBase {

    private ArmSubsystem arm;
    private double targetDeg;
    private Supplier<Double> targeter = null;
    private PIDController pid;

    public ArmToDegree(ArmSubsystem arm, double targetDegrees) {
        this.arm = arm;
        this.targetDeg = targetDegrees;
        pid = new PIDController(ArmConstants.ANGLE_P, ArmConstants.ANGLE_I, ArmConstants.ANGLE_D);
        pid.setTolerance(ArmConstants.pidPositionToleranceDegrees);
        addRequirements(arm);
    }

    public ArmToDegree(ArmSubsystem arm, Supplier<Double> targetDegrees) {
        this(arm, 0);
        this.targeter = targetDegrees;
    }

    @Override
    public void initialize() {
        if (targeter != null)
            this.targetDeg = targeter.get();
        pid.setSetpoint(this.targetDeg);
    }

    @Override
    public void execute() {
        double calculatedSpeed = pid.calculate(arm.getRotationDegrees());
        calculatedSpeed = MathUtil.clamp(calculatedSpeed, -ArmConstants.MAX_ROTATION_SPEED,
                ArmConstants.MAX_ROTATION_SPEED);
        arm.setRotatingMotor(calculatedSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setRotatingMotor(0);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}

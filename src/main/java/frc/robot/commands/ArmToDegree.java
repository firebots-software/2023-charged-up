package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ArmToDegree extends CommandBase{

    private ArmSubsystem arm;
    private double targetDeg;
    private PIDController pid;

    public ArmToDegree(ArmSubsystem arm, double targetDegrees){
        this.arm = arm;
        this.targetDeg = targetDegrees;
        pid = new PIDController(ArmConstants.ANGLE_P, ArmConstants.ANGLE_I, ArmConstants.ANGLE_D);
        pid.setTolerance(ArmConstants.pidPositionToleranceDegrees);
        addRequirements(arm);
    }
    
    @Override
    public void initialize(){
        pid.setSetpoint(this.targetDeg);
        // arm._frictionBreakOff();
    }

    @Override
    public void execute(){
        double calculatedSpeed = pid.calculate(arm.getRotationDegrees());
        calculatedSpeed = MathUtil.clamp(calculatedSpeed, -ArmConstants.MAX_ROTATION_SPEED, ArmConstants.MAX_ROTATION_SPEED);
        arm.setRotatingMotor(calculatedSpeed);
        System.out.println("going!");
    }

    @Override
    public void end(boolean interrupted){
        arm.setRotatingMotor(0);
        // arm._frictionBreakOn();
    }

    @Override
    public boolean isFinished(){
        return pid.atSetpoint();
    }
}

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
        pid = new PIDController(ArmConstants.angleP, ArmConstants.angleI, ArmConstants.angleD);
        pid.setTolerance(ArmConstants.pidPositionToleranceDegrees);
        addRequirements(arm);
    }
    
    @Override
    public void initialize(){
        pid.setSetpoint(this.targetDeg);
    }

    @Override
    public void execute(){
        double calculatedSpeed = pid.calculate(arm.getRotationDegrees());
        calculatedSpeed = MathUtil.clamp(calculatedSpeed, -0.5, 0.5);
        arm.setRotatingMotor(calculatedSpeed);
    }

    @Override
    public void end(boolean interrupted){
        arm.setRotatingMotor(0);
    }

    @Override
    public boolean isFinished(){
        return pid.atSetpoint();
    }
}

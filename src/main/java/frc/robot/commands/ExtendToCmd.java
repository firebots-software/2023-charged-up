package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendToCmd extends CommandBase {
    private ArmSubsystem arm;
    private double targetInches;
    private PIDController pid;

    public ExtendToCmd(ArmSubsystem arm, double targetInches){
        this.arm = arm;
        this.targetInches = targetInches;
        pid = new PIDController(ArmConstants.EXTENTION_P, ArmConstants.EXTENTION_I, ArmConstants.EXTENTION_D);
        pid.setTolerance(ArmConstants.pidExtentionToleranceInches);
        addRequirements(arm);
    }
    
    @Override
    public void initialize(){
        pid.setSetpoint(this.targetInches);
    }

    @Override
    public void execute(){
        double calculatedSpeed = pid.calculate(arm.getArmLength());
        calculatedSpeed = MathUtil.clamp(calculatedSpeed, -ArmConstants.MAX_EXTENTION_SPEED, ArmConstants.MAX_EXTENTION_SPEED);
        arm.setExtendingMotor(calculatedSpeed);
    }

    @Override
    public void end(boolean interrupted){
        arm.setExtendingMotor(0);
    }

    @Override
    public boolean isFinished(){
        return pid.atSetpoint();
    }
}

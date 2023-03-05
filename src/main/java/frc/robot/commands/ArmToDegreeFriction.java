package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ArmToDegreeFriction extends CommandBase{

    private ArmSubsystem arm;
    private double targetDeg;
    private double stopAt;
    private double direction;

    private boolean done = false;

    public ArmToDegreeFriction(ArmSubsystem arm, double targetDegrees, double stopBefore) {
        this.arm = arm;
        this.targetDeg = targetDegrees;

        this.direction = Math.signum(targetDeg - arm.getRotationDegrees());
        this.stopAt = targetDegrees - (direction * stopBefore);

        addRequirements(arm);
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double current = arm.getRotationDegrees();

        if (stopAt*direction < current*direction) {
            done = true;
            return;
        }

        double calculatedSpeed = (targetDeg - arm.getRotationDegrees()) * ArmConstants.angleP + (ArmConstants.FRICTION_BREAK_DEADBAND * direction);
        calculatedSpeed = MathUtil.clamp(calculatedSpeed, -ArmConstants.MAX_ROTATION_SPEED, ArmConstants.MAX_ROTATION_SPEED);
        arm.setRotatingMotor(calculatedSpeed);
    }

    @Override
    public void end(boolean interrupted){
        arm.setRotatingMotor(0);
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}

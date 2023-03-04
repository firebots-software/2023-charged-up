package frc.robot.commands;

// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RunMotor extends CommandBase {
    ArmSubsystem clawAndArm2;
    /** Creates a new RunMotor. */
    public RunMotor() {
      clawAndArm2 = ArmSubsystem.getInstance();
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        clawAndArm2.setRotatingMotor(0.2);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      clawAndArm2.setRotatingMotor(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
     return false;
    }
  }
  

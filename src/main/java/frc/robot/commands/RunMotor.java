package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotor extends CommandBase {
    WPI_TalonFX talon;
    /** Creates a new RunMotor. */
    public RunMotor(int id) {
      talon = new WPI_TalonFX(id);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("runing command");
        talon.set(0.2);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      talon.set(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  

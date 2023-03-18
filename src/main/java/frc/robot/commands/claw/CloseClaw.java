package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawSubsystem;

public class CloseClaw extends CommandBase {
    ClawSubsystem claw;
    public CloseClaw(ClawSubsystem p) {
      this.claw = p;
    }
    @Override
    public void initialize() {
        claw.close();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

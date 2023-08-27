package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SampleCmd extends CommandBase {
    // private subsystem instances or internal variables go here.

    public SampleCmd() {
        // initialize your command,
        // set internal variables with constructor args,
        // or set subsystem instances. 

        // put any subsystems this command uses as arguments to addRequirements.
        // this ensures that two commands running at the same time 
        // won't fight over control of the same subsystem.
        addRequirements();
    }

    @Override
    public void initialize() {
        // what does the command immediately do when first told to run?
    }
    
    @Override
    public void execute() {
        // what does the command do every ~1/50th of a second while running?
    }

    @Override
    public boolean isFinished() {
        // when does the command stop itself?
        // does it run only initialize and exit? does it never stop by itself?
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // what does the command do when told to stop (either by itself or externally)?
    }
}

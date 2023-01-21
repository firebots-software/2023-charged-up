package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SingleMotor;

public class PortNumCmd extends CommandBase {
    private int portNum = 0;
    private SingleMotor motor;

    private Supplier<Boolean> incrementer;
    private Supplier<Boolean> decrementer;
    private Supplier<Boolean> runner;
    private Supplier<Boolean> stopper;

    public PortNumCmd(Supplier<Boolean> incrementer, Supplier<Boolean> decrementer, Supplier<Boolean> runner, Supplier<Boolean> stopper) {
        this.incrementer = incrementer;
        this.decrementer = decrementer;
        this.runner = runner;
        this.stopper = stopper;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (incrementer.get()) {
            portNum++;
            changeMotor(portNum);
        }
        if (decrementer.get()) {
            portNum--;
            changeMotor(portNum);
        }
        if (runner.get()) {
            motor.run();
        }
        if (stopper.get()) {
            motor.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        motor.destroy();
    }

    private void changeMotor(int portNum) {
        motor.stop();
        motor.destroy();
        
        motor = new SingleMotor(portNum);

        System.out.println("Changed to port # " + portNum);
        SmartDashboard.putNumber("Port #", portNum);
    }
    
}

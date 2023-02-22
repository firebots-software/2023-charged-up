// package frc.robot.commands;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.subsystems.SingleMotor;

// public class PortNumCmd extends CommandBase {
//     private Supplier<Boolean> incrementer;
//     private Supplier<Boolean> decrementer;
//     private Supplier<Boolean> runner;
//     private Supplier<Boolean> stopper;

//     private int portNum = 0;
//     private SingleMotor motor;
//     private SingleMotor[] motors;

//     public PortNumCmd(Supplier<Boolean> incrementer, Supplier<Boolean> decrementer, Supplier<Boolean> runner, Supplier<Boolean> stopper, Subsystem defaultSystem) {
//         this.incrementer = incrementer;
//         this.decrementer = decrementer;
//         this.runner = runner;
//         this.stopper = stopper;

//         motors = new SingleMotor[16];
//         for (int i = 0; i < motors.length; i++) {
//             motors[i] = new SingleMotor(i);
//         }

//         changeMotor(portNum);

//         addRequirements(defaultSystem);
//     }

//     @Override
//     public void initialize() {}

//     @Override
//     public void execute() {
//         if (incrementer.get()) {
//             portNum = (portNum + 1) % 16;
//             changeMotor(portNum);
//         }
//         if (decrementer.get()) {
//             portNum = (portNum + 15) % 16;
//             changeMotor(portNum);
//         }
//         if (runner.get()) {
//             motor.run();
//         }
//         if (stopper.get()) {
//             motor.stop();
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {}

//     private void changeMotor(int portNum) {
//         motor = motors[portNum];

//         System.out.println("Changed to port # " + portNum);
//         SmartDashboard.putNumber("Port #", portNum);
//     }
    
// }

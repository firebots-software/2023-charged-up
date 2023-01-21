package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelOrientationCmd extends CommandBase {

    private final List<TestModule> modules;
    private int curModuleIndex = 0;

    private TestModule module;

    private Supplier<Boolean> incrementer;
    private Supplier<Boolean> decrementer;
    private Supplier<Double> angler;
    private Supplier<Boolean> printer;

    public WheelOrientationCmd(List<TestModule> modules, Supplier<Boolean> incrementer, Supplier<Boolean> decrementer, Supplier<Double> angler, Supplier<Boolean> printer, Subsystem defaultSystem) {
        this.modules = modules;

        this.incrementer = incrementer;
        this.decrementer = decrementer;
        this.angler = angler;
        this.printer = printer;
        
        changeModule(curModuleIndex);

        addRequirements(defaultSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (incrementer.get()) {
            curModuleIndex = (curModuleIndex + 1) % modules.size();
            changeModule(curModuleIndex);
        }
        if (decrementer.get()) {
            curModuleIndex = (curModuleIndex + modules.size() - 1) % modules.size();
            changeModule(curModuleIndex);
        }

        double by = angler.get();
        module.turn(Math.abs(by) > Constants.OI.DEADBAND ? by * 0.2 : 0);

        if (printer.get()) {
            module.print();
        }
    }

    private void changeModule(int curModuleIndex) {
        module = modules.get(curModuleIndex);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}


    public static class TestModule extends SubsystemBase {
        private final WPI_TalonFX driveMotor;
        private final WPI_TalonFX turnMotor;
        private final CANCoder canCoder;

        private PIDController pid;

        public TestModule(int driveMotorId, int turnMotorId, int canCoderId) {
            // we assume it is reversed
            driveMotor = new WPI_TalonFX(driveMotorId);
            turnMotor = new WPI_TalonFX(turnMotorId);
            canCoder = new CANCoder(canCoderId);

            CANCoderConfiguration canconfig = new CANCoderConfiguration();
            canconfig.sensorCoefficient = 1d / 4096d;
            canconfig.unitString = "rots";
            canconfig.sensorDirection = true;
            canCoder.configAllSettings(canconfig);

            canCoder.setPosition(0);

            pid = new PIDController(1, 0, 0);
            pid.enableContinuousInput(-1d, 1);
        }

        public void run() {
            driveMotor.set(0.2);
        }

        public void turn(double by) {
            turnMotor.set(by);
        }

        public void print() {
            System.out.println("abs enc. pos = " + canCoder.getAbsolutePosition());
            SmartDashboard.putNumber("abs enc. pos", canCoder.getAbsolutePosition());
        }
    }
}

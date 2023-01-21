package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotor extends SubsystemBase {
    private WPI_TalonFX motor;
    private boolean active;

    public SingleMotor(int port) {
        motor = new WPI_TalonFX(port);
        active = true;
    }

    public void run() {
        if (active) motor.set(0.2);
    }

    public void stop() {
        if (active) motor.set(0);
    }

    public void destroy() {
        if (!active) return;
        motor.DestroyObject();
        active = false;
    }
}

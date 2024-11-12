package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Krakens extends SubsystemBase {
    TalonFX motor1;
    TalonFX motor2;

    // constructor
    public Krakens(int id1, int id2) {
        this.motor1 = new TalonFX(id1);
        this.motor2 = new TalonFX(id2);
    }

    // other methods
    public void spinMotors(double speed) {
        this.motor1.set(speed);
        this.motor2.set(speed);
    }

    // eg: public void spin(int speed) {motor.setSpeed(speed.)}
    
}

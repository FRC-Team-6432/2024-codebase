package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Krakens extends SubsystemBase {
    public TalonFX motor1;
    public TalonFX motor2;
    public DigitalInput limit = new DigitalInput(1);

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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("limit switch", limit.get());
    }

    

    // eg: public void spin(int speed) {motor.setSpeed(speed.)}
    
}

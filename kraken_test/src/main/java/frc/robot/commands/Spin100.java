package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Krakens;

public class Spin100 extends Command {
    // attributes
    Krakens kraken_subsystem;
    Timer timer = new Timer();

    // constructor
    public Spin100(Krakens krakens_subsystem) {
        this.kraken_subsystem = krakens_subsystem;
    }

    @Override
    public void initialize() {
        this.timer.start();
        this.kraken_subsystem.spinMotors(1);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("timer", this.timer.get());
    }

    @Override
    public boolean isFinished() {
        if (!this.kraken_subsystem.limit.get()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.kraken_subsystem.motor1.set(0);
        this.kraken_subsystem.motor2.set(0);
    }

}

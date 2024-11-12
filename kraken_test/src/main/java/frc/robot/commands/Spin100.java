package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Krakens;

public class Spin100 extends Command {
    // attributes
    Krakens kraken_subsystem;

    // constructor
    public Spin100(Krakens krakens_subsystem) {
        this.kraken_subsystem = krakens_subsystem;
    }

    @Override
    public void execute() {
        this.kraken_subsystem.spinMotors(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

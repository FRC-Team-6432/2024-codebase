package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class StopMotor extends Command {
    private final MotorSubsystem m_subsystem;

    public StopMotor(MotorSubsystem m_subsystem) {
        this.m_subsystem = m_subsystem;

        addRequirements(m_subsystem);

    }

    @Override
    public void initialize() {
        this.m_subsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }






}

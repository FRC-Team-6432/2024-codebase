package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class SetSpeed extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MotorSubsystem mSubsystem;
    private final DoubleSupplier speedSupplier;

    public SetSpeed(MotorSubsystem mSubsystem, DoubleSupplier speedSupplier) {
        this.mSubsystem = mSubsystem;
        this.speedSupplier = speedSupplier;
        addRequirements(mSubsystem);
    }

    @Override
    public void execute() {
        this.mSubsystem.setSpeed(this.speedSupplier.getAsDouble());
        SmartDashboard.putNumber("Speed", this.speedSupplier.getAsDouble());
    }
}

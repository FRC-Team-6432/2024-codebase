// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmUpdated;
import edu.wpi.first.wpilibj.Timer;

public class AutoDown extends Command {
  ArmUpdated arm;
  Timer timer = new Timer();
  Boolean finish;
  /** Creates a new IntakeDown. */
  public AutoDown(ArmUpdated urm) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = urm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    arm.setArmToIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (timer.get()<0.5){
      finish = false;
    }
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

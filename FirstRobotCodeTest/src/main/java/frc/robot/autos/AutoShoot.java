// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */
  Intake intake;
  Timer timer = new Timer();
  private boolean finish = false;
  public AutoShoot(Intake intoke) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intoke;
    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while (timer.get() < 2){
      intake.autoyeet();
    }
    intake.stop();
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

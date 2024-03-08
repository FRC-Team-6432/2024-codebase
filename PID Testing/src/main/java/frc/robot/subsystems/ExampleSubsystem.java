// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  DutyCycleEncoder encoder = new DutyCycleEncoder(3);
  WPI_TalonSRX motorLeft = new WPI_TalonSRX(2);
  WPI_TalonSRX motorRight = new WPI_TalonSRX(1);
  // WPI_TalonSRX motorTalon = new WPI_TalonSRX(2);
  // WPI_TalonSRX motorTalon2 = new WPI_TalonSRX(3);

  boolean xWasPressed = false;
  boolean yWasPressed = false;
  boolean bWasPressed = false;
  boolean aWasPressed = false;

  int current_value = 0;
  
  XboxController xbox = new XboxController(0);

  double kp = 0;
  double ki = 0;
  double kd = 0;

  double[] values = {kp, ki, kd};
  String[] valuesToString = {"kp", "ki", "kd"};

  double delta_k = 1;

  PIDController pidController = new PIDController(kp, ki, kd);

  double position = 0;

  public ExampleSubsystem() {
    motorRight.setInverted(true);
    motorLeft.follow(motorRight);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (xbox.getRightTriggerAxis() > 0.9 ) {
      position += 0.01;
      if (position > 1) {position = 1;}
    }
    else if (xbox.getLeftTriggerAxis() > 0.9) {
      position -= 0.01;
      if (position < 0) {position = 0;}
    }

    double absPos = encoder.getAbsolutePosition();
    motorRight.set(pidController.calculate(encoder.getDistance(), position));
    // motorTalon.set(pidController.calculate(encoder.getDistance(), position));

    if (!aWasPressed && xbox.getAButtonPressed()) {
      values[current_value] -= delta_k;
      if (values[current_value] < 0) {values[current_value] = 0;}

      if (current_value == 0) {
        pidController.setP(values[current_value]);
      }
      else if (current_value == 1) {
        pidController.setI(values[current_value]);
      }
      else if (current_value == 2) {
        pidController.setD(values[current_value]);
      }


      aWasPressed = true;
    }
    if (xbox.getAButtonReleased()) {
      aWasPressed = false;
    }

    if (!yWasPressed && xbox.getYButtonPressed()) {
      values[current_value] += delta_k;

      if (current_value == 0) {
        pidController.setP(values[current_value]);
      }
      else if (current_value == 1) {
        pidController.setI(values[current_value]);
      }
      else if (current_value == 2) {
        pidController.setD(values[current_value]);
      }

      yWasPressed = true;
    }
    if (xbox.getYButtonReleased()) {
      yWasPressed = false;
    }

    if (!xWasPressed && xbox.getXButtonPressed()) {
      current_value += -1;
      if (current_value < 0) {
        current_value = 2;
      }
      xWasPressed = true;
    }
    if (xbox.getXButtonReleased()) {
      xWasPressed = false;
    }

    if (!bWasPressed && xbox.getBButtonPressed()) {
      current_value += 1;
      if (current_value > 2) {
        current_value = 0;
      }
      bWasPressed = true;
    }
    if (xbox.getBButtonReleased()) {
      bWasPressed = false;
    }

    SmartDashboard.putNumber("encoder position: ", absPos);
    SmartDashboard.putNumber("set position", position);
    SmartDashboard.putNumber("kp: ", values[0]);
    SmartDashboard.putNumber("ki: ", values[1]);
    SmartDashboard.putNumber("kd: ", values[2]);
    SmartDashboard.putString("Currently Changing: ", valuesToString[current_value]);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

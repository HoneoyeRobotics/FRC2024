// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}
public boolean note = false;
public VictorSPX topmotor = new VictorSPX(10);
public VictorSPX bottemmotor = new VictorSPX(11);
public AnalogInput noteSensor = new AnalogInput(0);
public void runbottommotor (double speed) {
  if ((speed < 0.05 ) && (speed > -0.05)) speed = 0;
  bottemmotor.set(ControlMode.PercentOutput, speed);
}
public void runtopmotor (double speed) {
  if ((speed < 0.05 ) && (speed > -0.05)) speed = 0;
  topmotor.set(ControlMode.PercentOutput, speed *-1);
}

public boolean noteCheck() {
    if (noteSensor.getValue() < 500 && noteSensor.getValue() > 230) note = true;
    else note = false;
  return note;
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
    SmartDashboard.putNumber("Sensor", noteSensor.getValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {

    topmotor = new CANSparkMax(Constants.ShooterConstants.TopMotorCanID, MotorType.kBrushless);
    bottommotor = new CANSparkMax(Constants.ShooterConstants.BottomMotorCanID, MotorType.kBrushless);
    topmotor.setInverted(false);
    bottommotor.setInverted(false);
    TheThingThatStabsTheNote = new VictorSPX(Constants.ShooterConstants.TheThingThatStabsTheNoteCanID);
    StabPosition = new Encoder(Constants.ShooterConstants.NoteSensorAInput,
        Constants.ShooterConstants.NoteSensorBInput);
    StabPosition.reset();
    stabPidController = new PIDController(0.04, 0, 0.001);
    stabPidController.setTolerance(1);
    stabPidController.setSetpoint(0);
    topmotor.setIdleMode(IdleMode.kBrake);
    bottommotor.setIdleMode(IdleMode.kBrake);
  }

  public PIDController stabPidController;
  public boolean note = false;
  public CANSparkMax topmotor;
  public CANSparkMax bottommotor;
  public VictorSPX TheThingThatStabsTheNote;
  public AnalogInput noteSensor = new AnalogInput(Constants.ShooterConstants.NoteSensorInput);
  public Encoder StabPosition;

  public void RunTheThingThatStabsTheNote(double speed) {
    TheThingThatStabsTheNote.set(VictorSPXControlMode.PercentOutput, speed);

  }

  public int getStabPosition() {
    return StabPosition.get();
  }

  public void runbottommotor(double speed) {
    if ((speed < 0.05) && (speed > -0.05))
      speed = 0;
    bottommotor.set(speed);
  }

  public void runtopmotor(double speed) {
    if ((speed < 0.05) && (speed > -0.05))
      speed = 0;
    topmotor.set(speed);
  }

  private boolean stabenabled = true;

  public void toggleStabPID() {
    stabenabled = !stabenabled;
  }

  public boolean noteCheck() {
    if (noteSensor.getValue() < 500 && noteSensor.getValue() > 230)
      note = true;
    else
      note = false;
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
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
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
    // SmartDashboard.putNumber("Sensor", noteSensor.getValue());
    // SmartDashboard.putNumber("Stab Position", getStabPosition());
    // SmartDashboard.putNumber("Stab Setpoint", stabPidController.getSetpoint());

    // if (stabPidController.getP() != RobotPrefs.getStabP())
    // stabPidController.setP(RobotPrefs.getStabP());

    // if (stabPidController.getI() != RobotPrefs.getStabI())
    // stabPidController.setI(RobotPrefs.getStabI());

    // if (stabPidController.getD() != RobotPrefs.getStabD())
    // stabPidController.setD(RobotPrefs.getStabD());
    if (stabenabled == true) {
      double stabbyspeed = stabPidController.calculate(getStabPosition());
      RunTheThingThatStabsTheNote(stabbyspeed);
    }
  }

  public void setStabPosition(double pos) {
    stabPidController.setSetpoint(pos);
  }

  public boolean atStabPosition() {
    boolean atset = stabPidController.atSetpoint();
    // if(atset && stabPidController.getSetpoint() != 0)
    // stabPidController.setSetpoint(0);
    return atset;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

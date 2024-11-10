// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax LeftClimber;
  private CANSparkMax RightClimber;

  private PIDController LeftClimbPID;
  private PIDController RightClimbPID;

  /** Creates a new Climber. */
  public Climber() {
    LeftClimber = new CANSparkMax(ClimberConstants.LeftClimber, MotorType.kBrushless);
    RightClimber = new CANSparkMax(ClimberConstants.RightClimber, MotorType.kBrushless);
    LeftClimber.setIdleMode(IdleMode.kBrake);
    RightClimber.setIdleMode(IdleMode.kBrake);
    LeftClimber.getEncoder().setPosition(0.0);
    RightClimber.getEncoder().setPosition(0.0);



    
    LeftClimbPID = new PIDController(0.2, 0, 0);
    RightClimbPID = new PIDController(0.2, 0, 0);
    LeftClimbPID.setTolerance(1);
    RightClimbPID.setTolerance(1);
    
    LeftClimbPID.setSetpoint(0.5);
    RightClimbPID.setSetpoint(-0.5);
  }

  public void holdClimbersHere(){
    LeftClimbPID.setSetpoint(LeftClimber.getEncoder().getPosition());
    RightClimbPID.setSetpoint(RightClimber.getEncoder().getPosition());
  }


  public boolean holdClimber = true;
  public void resetLeftClimberEncoder(){
    LeftClimber.getEncoder().setPosition(0);
  }
  public void resetRightClimberEncoder(){
    RightClimber.getEncoder().setPosition(0);
  }
  public void setLeftClimber(double speed) {
    LeftClimber.set(speed);

    SmartDashboard.putNumber("LeftClimbSpeed", speed);
  }

  public void setRightClimber(double speed) {
    RightClimber.set(speed);

    SmartDashboard.putNumber("RightClimbSpeed", speed);
  }

  public double getRightClimberPosition() {
    SmartDashboard.putNumber("RightClimbPos", RightClimber.getEncoder().getPosition());
    return RightClimber.getEncoder().getPosition();

  }

  public double getLeftClimberPosition() {
    SmartDashboard.putNumber("LeftClimbPos", LeftClimber.getEncoder().getPosition());
    return LeftClimber.getEncoder().getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("LeftClimbPos", LeftClimber.getEncoder().getPosition());
    // SmartDashboard.putNumber("RightClimbPos", RightClimber.getEncoder().getPosition()); 

    //check climber position
SmartDashboard.putBoolean("hold Climber", holdClimber);

    if(holdClimber == true){
      setLeftClimber(LeftClimbPID.calculate(getLeftClimberPosition()));
      setRightClimber(RightClimbPID.calculate(getRightClimberPosition()));
    }
  }
}

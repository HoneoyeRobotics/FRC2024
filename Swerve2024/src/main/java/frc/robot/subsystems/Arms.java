// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositions;

public class Arms extends SubsystemBase {
  /** Creates a new Arms. */

  private CANSparkMax shoulderMotor = new CANSparkMax(Constants.ArmConstants.ShoulderCanID, MotorType.kBrushless);
  private CANSparkMax elbowMotor = new CANSparkMax(Constants.ArmConstants.ElbowCanID, MotorType.kBrushless);

  private boolean PIDEnabled = true;
  private PIDController shoulderPIDContrller;
  private PIDController elbowPIDContrller;
  private DigitalInput ArmLimitSwitch = new DigitalInput(0);
  private double shoulderMotorSetpoint = 0.0;
  private double elbowMotorSetpoint = 0.0;

  private ArmPosition prevPosition = ArmPosition.Home;
  private ArmPosition armPosition = ArmPosition.Home;

  public Arms() {

    shoulderMotor.getEncoder().setPosition(0.0);
    elbowMotor.getEncoder().setPosition(0.0);

    shoulderPIDContrller = new PIDController(0.4, 0, 0);
    shoulderPIDContrller.setSetpoint(0.0);
    shoulderPIDContrller.setTolerance(0.5);

    elbowPIDContrller = new PIDController(0.3, 0, 0);
    elbowPIDContrller.setSetpoint(0.0);
    elbowPIDContrller.setTolerance(0.5);

    shoulderMotor.setIdleMode(IdleMode.kBrake);
    elbowMotor.setIdleMode(IdleMode.kBrake);

    shoulderMotor.setSmartCurrentLimit(40);
    elbowMotor.setSmartCurrentLimit(40);

    // elbowMotor.setInverted(true);
  }

  public void setArmPosition(ArmPosition armPosition) {
    prevPosition = this.armPosition;
    this.armPosition = armPosition;
    SmartDashboard.putString("Arm Position", armPosition.toString());
    SmartDashboard.putString("Prev Position", prevPosition.toString());
  }

  public ArmPosition getArmPosition() {
    return armPosition;
  }

  
  public ArmPosition getPreviousPosition() {
    return prevPosition;
  }


  public void FixArmInTeleop(){
    switch(armPosition){
      case ToAmp:
      case ToClimber:
      case ToFeeder:
      case ToSpeaker:
      case ToHome:
      case ToPickup:
        armPosition = prevPosition;
        break;
    }
  }

  public void setTolerance(double tolerance) {
    elbowPIDContrller.setTolerance(tolerance);
    shoulderPIDContrller.setTolerance(tolerance);
  }

  public boolean getArmLimitSwitch() {
    return !(ArmLimitSwitch.get());
  }

  public void togglePID() {
    PIDEnabled = !PIDEnabled;
    if (PIDEnabled == true) {
      resetallpositions();
    }
  }

  public void resetallpositions() {
    shoulderMotorSetpoint = shoulderMotor.getEncoder().getPosition();
    elbowMotorSetpoint = elbowMotor.getEncoder().getPosition();

  }

  public void resetshoulderEncoder() {
    shoulderMotor.getEncoder().setPosition(0.0);
    shoulderMotorSetpoint = 0;
  }

  public void resetelbowEncoder() {
    elbowMotor.getEncoder().setPosition(0.0);
    elbowMotorSetpoint = 0;
  }

  public double getElbowPosition() {
    return elbowMotor.getEncoder().getPosition();
  }

  public double getShoulderPosition() {
    return shoulderMotor.getEncoder().getPosition();
  }

  public void moveshoulder(double position, boolean setPosition) {

    if (setPosition)
      shoulderMotorSetpoint = position;
    else
      shoulderMotorSetpoint += position;
  }

  public void moveelbow(double position, boolean setPosition) {

    if (setPosition)
      elbowMotorSetpoint = position;
    else
      elbowMotorSetpoint += position;
  }

  public boolean iselbowAtPosition() {
    return elbowPIDContrller.atSetpoint();
  }

  public boolean isshoulderAtPosition() {
    return shoulderPIDContrller.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  //  SmartDashboard.putBoolean("PID Enabled", PIDEnabled);

    SmartDashboard.putBoolean("ElbowAtSetpoint", elbowPIDContrller.atSetpoint());
    SmartDashboard.putBoolean("ShoulderAtSetpoint", shoulderPIDContrller.atSetpoint());

    // SmartDashboard.putNumber("ElbowTol",
    // elbowPIDContrller.getPositionTolerance());
    // SmartDashboard.putNumber("ShoulderTol",
    // shoulderPIDContrller.getPositionTolerance());

    SmartDashboard.putNumber("elbowSetpoint", elbowMotorSetpoint);
    SmartDashboard.putNumber("elbowPosition", getElbowPosition());

    SmartDashboard.putNumber("shoulderPosition", getShoulderPosition());
    SmartDashboard.putNumber("shoulderSetpoint", shoulderMotorSetpoint);

    SmartDashboard.putBoolean("Arm Limit Switch", getArmLimitSwitch());
    if (PIDEnabled) {

      // if (shoulderPIDContrller.getP() != RobotPrefs.getShoulderP())
      // shoulderPIDContrller.setP(RobotPrefs.getShoulderP());

      // if (shoulderPIDContrller.getI() != RobotPrefs.getShoulderI())
      // shoulderPIDContrller.setI(RobotPrefs.getShoulderI());

      // if (shoulderPIDContrller.getD() != RobotPrefs.getShoulderD())
      // shoulderPIDContrller.setD(RobotPrefs.getShoulderD());

      // if (elbowPIDContrller.getP() != RobotPrefs.getElbowP())
      // elbowPIDContrller.setP(RobotPrefs.getElbowP());

      // if (elbowPIDContrller.getI() != RobotPrefs.getElbowI())
      // elbowPIDContrller.setI(RobotPrefs.getElbowI());

      // if (elbowPIDContrller.getD() != RobotPrefs.getElbowD())
      // elbowPIDContrller.setD(RobotPrefs.getElbowD());

      double shoulderRotateSpeed = shoulderPIDContrller.calculate(getShoulderPosition(), shoulderMotorSetpoint);
      double elbowRotateSpeed = elbowPIDContrller.calculate(getElbowPosition(), elbowMotorSetpoint);
      if (getArmLimitSwitch() && 
          shoulderRotateSpeed < 0 && 
          getShoulderPosition() <= 1 && 
          shoulderMotorSetpoint <= 0) {
        shoulderRotateSpeed = 0;
        shoulderMotorSetpoint = 0;
        resetshoulderEncoder();
      }

      boolean Shoulderclose = Math.abs(shoulderMotorSetpoint - getShoulderPosition()) < 2;
      boolean Elbowclose = Math.abs(elbowMotorSetpoint - getElbowPosition()) < 2;

      if (shoulderRotateSpeed > 0 && Shoulderclose == false)
        shoulderRotateSpeed = 0.33;
      if (shoulderRotateSpeed < 0 && Shoulderclose == false)
        shoulderRotateSpeed = -0.33;

      if (elbowRotateSpeed > 0 && Elbowclose == false)
        elbowRotateSpeed = 0.22;
      if (elbowRotateSpeed < 0 && Elbowclose == false)
        elbowRotateSpeed = -0.45;

      if (elbowRotateSpeed > 0 && elbowMotorSetpoint == 0 && getElbowPosition() > -4)
        elbowRotateSpeed = 0;

      // if (elbowRotateSpeed < 0 &&
      //     armPosition == ArmPosition.ToPickup &&
      //     elbowMotorSetpoint == ArmPositions.PickupE &&
      //     shoulderMotorSetpoint == ArmPositions.PickupS &&
      //     getElbowPosition() < -5){       
      //       elbowRotateSpeed = 0;
      //     //  System.out.println("Cut elbow power.");
      //     }

      if (shoulderRotateSpeed > 0 &&
          armPosition == ArmPosition.ToPickup &&
          elbowMotorSetpoint == ArmPositions.PickupE &&
          shoulderMotorSetpoint == ArmPositions.PickupS &&
          getShoulderPosition() > 14)   
          {       
            shoulderRotateSpeed = 0;
         //  System.out.println("Cut shoulder power.");
          }
      shoulderMotor.set(shoulderRotateSpeed);
      elbowMotor.set(elbowRotateSpeed);
      // SmartDashboard.putNumber("shoulderSpeed", shoulderRotateSpeed);
      // SmartDashboard.putNumber("elbowSpeed", elbowRotateSpeed);

    } else {
      // SmartDashboard.putNumber("shoulderSpeed", 0);
      // SmartDashboard.putNumber("elbowSpeed", 0);
      shoulderMotor.set(0);
      elbowMotor.set(0);
    }

  }
}

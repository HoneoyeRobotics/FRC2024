// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomDrives extends SubsystemBase {

  public WPI_VictorSPX leftMototor = new WPI_VictorSPX(10);
  public WPI_VictorSPX rightMotor = new WPI_VictorSPX(11);
  
  public DifferentialDrive bottomDrive = new DifferentialDrive(leftMototor, rightMotor);


  public void drive(double xSpeed, double zRotation){
    bottomDrive.arcadeDrive(xSpeed, zRotation);
  }
  /** Creates a new BottomDrives. */
  public BottomDrives() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

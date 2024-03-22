// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends ParallelCommandGroup {
  /** Creates a new RunShooter. */
  public ShootSequence(Shooter shooter, double shooterWait, double shooterTime, double shooterSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand())
  
    addCommands(
    //run the shooter for x seconds.
      new RunShooter(shooter, shooterSpeed).withTimeout(shooterTime),

    //after y seconds, fire the relay. this allows the shooter to get up to speed before we fire.
      new SequentialCommandGroup(
          new WaitCommand(shooterWait),
          new FireTheThingThatStabsTheNote(shooter, RobotPrefs.getTheThingThatStabsTheNoteRot() * -1, false),
          new FireTheThingThatStabsTheNote(shooter, 0, true)
      )
    );
  }
}

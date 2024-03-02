// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPrefs;
import frc.robot.commands.ArmMovement.*;
import frc.robot.subsystems.Arms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmAmp extends InstantCommand {
  /** Creates a new ArmHome. */
  private Arms arms;

  public ArmAmp(Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arms = arms;
  }

  @Override
  public void initialize() {
    switch (arms.getArmPosition()) {
      case Home:
        new HomeToAmp(arms).schedule();
        break;
      case Speaker:
        new SpeakerToHome(arms).schedule();
        ;
        break;
      case Feeder:
        new FeederToHome(arms).schedule();
        break;
      case Pickup:
        new PickUpToAmp(arms).schedule();
        break;
      case Amp:
        break;
        case Climber:
        break;
    }
  }
}

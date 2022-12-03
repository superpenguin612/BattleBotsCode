// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class AutoCommand extends SequentialCommandGroup {
  public AutoCommand(Drivetrain drivetrain) {
    addCommands(
        new TurnToAngle(30, drivetrain), // 5 is in meters
        new DriveStraight(-1, drivetrain) // 5 is in meters
    );
  }
}
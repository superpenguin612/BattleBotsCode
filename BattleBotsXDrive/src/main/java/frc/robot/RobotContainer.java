// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    private final Joystick joystick = new Joystick(0);

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter thetaRateLimiter = new SlewRateLimiter(3);

    public RobotContainer() {
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> drivetrain.drive(
                                xRateLimiter.calculate(joystick.getX()), // FIXME might need to be inverted
                                yRateLimiter.calculate(joystick.getY()),
                                thetaRateLimiter.calculate(joystick.getTwist()),
                                true),
                        drivetrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand(); // does nothing
    }
}

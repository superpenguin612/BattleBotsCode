package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Turns to a certain gyro angle using a PID loop.
 */
public class TurnToAngle extends PIDCommand {
    private final Drivetrain drivetrain;

    public TurnToAngle(double setpoint, Drivetrain drivetrain) {
        super(new PIDController(1, 0, 0), drivetrain::getGyroAngle, setpoint,
                d -> drivetrain.tankDrive(d, -d),
                drivetrain);
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.drivetrain.resetEncoders();
        this.drivetrain.stop();
        this.drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}

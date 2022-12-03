// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    /**
     * These four objects are your motor controllers. They allow you to set the
     * speeds of your motors.
     */
    private CANSparkMax leftPrimaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.LEFT_PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax leftSecondaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.LEFT_SECONDARY,
            MotorType.kBrushless);
    private CANSparkMax rightPrimaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.RIGHT_PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax rightSecondaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.RIGHT_SECONDARY,
            MotorType.kBrushless);
    /**
     * These four objects are your motor controllers. If you are using TalonSRX motor controllers, here is how to initalize them.
     */
    // private TalonSRX leftPrimaryMotor = new TalonSRX(Constants.Drivetrain.CANIDs.LEFT_PRIMARY);
    // private TalonSRX leftSecondaryMotor = new TalonSRX(Constants.Drivetrain.CANIDs.LEFT_SECONDARY);
    // private TalonSRX rightPrimaryMotor = new TalonSRX(Constants.Drivetrain.CANIDs.RIGHT_PRIMARY);
    // private TalonSRX rightSecondaryMotor = new TalonSRX(Constants.Drivetrain.CANIDs.RIGHT_SECONDARY);

    /**
     * These two objects are "groups" of motor controllers. We typically group the
     * left and right sides of your drivetrain to make it easier to work with.
     */
    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftPrimaryMotor, leftSecondaryMotor);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightPrimaryMotor, rightSecondaryMotor);

    /**
     * The DifferentialDrive object is a special object in WPILib that provides
     * helper methods for tankDrive and arcadeDrive, saving you the trouble of
     * making them yourself.
     */
    private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    /**
     * The gyroscope that tells us the angle of the robot.
     */
    private AHRS navx = new AHRS(SerialPort.Port.kMXP);

    public Drivetrain() {
        leftPrimaryMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        rightPrimaryMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);

        // Your one side of your drivetrain will need to be inverted such that positive voltages result in going forward. 
        // If your robot is reversed, flip which motor is set to inverted.
        leftMotors.setInverted(Constants.Drivetrain.LEFT_IS_INVERTED);
        leftMotors.setInverted(Constants.Drivetrain.RIGHT_IS_INVERTED);
    }

    /**
     * Drives the robot with tank drive controls.
     * 
     * @param leftSpeed the speed [-1, 1] of the left side of the drivetrain
     * @param rightSpeed the speed [-1, 1] of the right side of the drivetrain
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Gets the current angle of the gyro.
     */
    public double getGyroAngle() {
        return navx.getAngle();
    }

    /**
     * Reset the navX angle.
     */
    public void resetGyro() {
        navx.reset();
    }

    /**
     * Gets the average position of the left and right encoders.
     * 
     * @return the average position of the left and right encoders.
     */
    public double getPosition() {
        return (double) (leftPrimaryMotor.getEncoder().getPosition() + rightPrimaryMotor.getEncoder().getPosition())
                / 2.0;
    }

    /**
     * Resets the drivetrain encoders by setting their positions to zero.
     */
    public void resetEncoders() {
        leftPrimaryMotor.getEncoder().setPosition(0);
        rightPrimaryMotor.getEncoder().setPosition(0);
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        tankDrive(0, 0);
    }

}

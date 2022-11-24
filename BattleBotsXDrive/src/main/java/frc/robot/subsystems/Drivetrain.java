// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Drivetrain subsystem, includes all of the motors and the methods with which
 * to drive the bot.
 */
public class Drivetrain extends SubsystemBase {
    private CANSparkMax frontLeftMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.FRONT_LEFT,
            MotorType.kBrushless);
    private CANSparkMax frontRightMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.FRONT_RIGHT,
            MotorType.kBrushless);
    private CANSparkMax backLeftMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.BACK_LEFT,
            MotorType.kBrushless);
    private CANSparkMax backRightMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.BACK_RIGHT,
            MotorType.kBrushless);
    private AHRS navx = new AHRS(SerialPort.Port.kMXP);

    /**
     * Initializes the drivetrain.
     */
    public Drivetrain() {
        frontLeftMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        frontRightMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        backLeftMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        backRightMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);

        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        frontLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        frontRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Convert x, y, and theta speeds into motor speeds and drive the robot.
     * 
     * @param xSpeed     the left/right speed of the drivetrain. left is positive.
     * @param ySpeed     the forward/backward speed of the drivetrain. forward is
     *                   positive.
     * @param thetaSpeed the rotational speed of the drivetrain. CCW is positive.
     */
    public void drive(double xSpeed, double ySpeed, double thetaSpeed, boolean isFieldOriented) {
        ChassisSpeeds chassisSpeeds = null;
        if (isFieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                    thetaSpeed, navx.getRotation2d());

            xSpeed = chassisSpeeds.vxMetersPerSecond; // not actually m/s
            ySpeed = chassisSpeeds.vyMetersPerSecond; // not actually m/s
            thetaSpeed = chassisSpeeds.omegaRadiansPerSecond; // not actually rad/s
        }

        double[] speeds = new double[] {
                ySpeed - xSpeed - thetaSpeed,
                ySpeed + xSpeed + thetaSpeed,
                ySpeed - xSpeed + thetaSpeed,
                ySpeed + xSpeed - thetaSpeed }; // front left, front right, back right, back left

        // it is possible for wheel speeds to go over 1, which is bad, so this
        // "desaturates" them.
        double[] desaturatedSpeeds = desaturateWheelSpeeds(speeds);

        frontLeftMotor.set(desaturatedSpeeds[0]);
        frontRightMotor.set(desaturatedSpeeds[1]);
        backRightMotor.set(desaturatedSpeeds[2]);
        backLeftMotor.set(desaturatedSpeeds[3]);

    }

    public static double[] desaturateWheelSpeeds(double[] speeds) {
        double maxSpeed = Double.MIN_VALUE;
        for (double speed : speeds) {
            if (speed > maxSpeed) {
                maxSpeed = speed;
            }
        }

        double[] desaturatedSpeeds = new double[4];
        if (maxSpeed > 1) {
            for (int i = 0; i < desaturatedSpeeds.length; i++) {
                desaturatedSpeeds[i] = speeds[i] / maxSpeed;
            }
        }
        return desaturatedSpeeds;
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        drive(0, 0, 0, false);
    }

    /**
     * Gets the current angle of the gyro.
     */
    public double getGyroAngle() {
        return navx.getAngle();
    }

    /**
     * Gets the current angle of the gyro.
     */
    public Rotation2d getGyroRotation2d() {
        return navx.getRotation2d();
    }

    /**
     * Reset the navX angle.
     */
    public void resetGyro() {
        navx.reset();
    }
}

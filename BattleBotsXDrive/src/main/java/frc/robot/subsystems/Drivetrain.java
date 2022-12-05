// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.loggers.SingleItemLogger;

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

    private double frontLeftMotorSpeed = 0.0;
    private double frontRightMotorSpeed = 0.0;
    private double backLeftMotorSpeed = 0.0;
    private double backRightMotorSpeed = 0.0;

    private double joystickX = 0.0;
    private double joystickY = 0.0;
    private double joystickTwist = 0.0;

    private double[] speeds = new double[4];
    private double[] desaturatedSpeeds = new double[4];

    /**
     * Initializes the drivetrain.
     */
    public Drivetrain() {
        LoggingManager.getInstance().addGroup("drivetrain",
                new LogGroup(new DeviceLogger<CANSparkMax>(frontLeftMotor, "Front Left Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(frontLeftMotor)),
                        new DeviceLogger<CANSparkMax>(frontRightMotor, "Front Right Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(frontRightMotor)),
                        new DeviceLogger<CANSparkMax>(backLeftMotor, "Back Left Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(backLeftMotor)),
                        new DeviceLogger<CANSparkMax>(backRightMotor, "Back Right Motor",
                                LogProfileBuilder.buildCANSparkMaxLogItems(backRightMotor)),
                        new DeviceLogger<AHRS>(navx, "NavX",
                                LogProfileBuilder.buildNavXLogItems(navx)),
                        new SendableLogger("NavX", navx),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Front Left Motor Speed",
                                () -> frontLeftMotorSpeed, LogLevel.MAIN),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Front Right Motor Speed",
                                () -> frontRightMotorSpeed, LogLevel.MAIN),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Back Left Motor Speed", () -> backLeftMotorSpeed),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Back Right Motor Speed",
                                () -> backRightMotorSpeed, LogLevel.MAIN),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Joystick X",
                                () -> joystickX, LogLevel.MAIN),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Joystick Y",
                                () -> joystickY, LogLevel.MAIN),
                        new SingleItemLogger<Double>(LogType.NUMBER, "Joystick Twist",
                                () -> joystickTwist, LogLevel.MAIN),
                        new SingleItemLogger<double[]>(LogType.NUMBER_ARRAY, "Speeds",
                                () -> speeds, LogLevel.MAIN),
                        new SingleItemLogger<double[]>(LogType.NUMBER_ARRAY, "Desaturated Speeds",
                                () -> desaturatedSpeeds, LogLevel.MAIN)

                ));

        frontLeftMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        frontRightMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        backLeftMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);
        backRightMotor.getEncoder().setPositionConversionFactor(Constants.Drivetrain.ENCODER_DISTANCE_TO_METERS);

        frontLeftMotor.setInverted(Constants.Drivetrain.Inversion.FRONT_LEFT);
        frontRightMotor.setInverted(Constants.Drivetrain.Inversion.FRONT_RIGHT);
        backLeftMotor.setInverted(Constants.Drivetrain.Inversion.BACK_LEFT);
        backRightMotor.setInverted(Constants.Drivetrain.Inversion.BACK_RIGHT);

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
        if (isFieldOriented) {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
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
        this.speeds = speeds;

        // it is possible for wheel speeds to go over 1, which is bad, so this
        // "desaturates" them.
        double[] desaturatedSpeeds = desaturateWheelSpeeds(speeds);
        this.desaturatedSpeeds = desaturatedSpeeds;

        frontLeftMotor.set(desaturatedSpeeds[0]);
        frontRightMotor.set(desaturatedSpeeds[1]);
        backRightMotor.set(desaturatedSpeeds[2]);
        backLeftMotor.set(desaturatedSpeeds[3]);

    }

    public static double[] desaturateWheelSpeeds(double[] speeds) {
        double maxSpeed = Double.MIN_VALUE;
        for (double speed : speeds) {
            double scalar = Math.abs(speed);
            if (scalar > maxSpeed) {
                maxSpeed = scalar;
            }
        }
        System.out.println(maxSpeed);

        double[] desaturatedSpeeds = new double[4];
        if (maxSpeed > 1) {
            for (int i = 0; i < desaturatedSpeeds.length; i++) {
                desaturatedSpeeds[i] = speeds[i] / maxSpeed;
            }
        } else {
            desaturatedSpeeds = speeds;
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

    /**
     * This is for debug purposes only.
     * 
     * @param frontLeft
     * @param frontRight
     * @param backLeft
     * @param backRight
     */
    public void setMotorSpeeds(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftMotor.set(frontLeft);
        frontRightMotor.set(frontRight);
        backLeftMotor.set(backLeft);
        backRightMotor.set(backRight);
    }
}

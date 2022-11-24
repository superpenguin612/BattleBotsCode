// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax leftPrimaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.LEFT_PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax leftSecondaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.LEFT_SECONDARY,
            MotorType.kBrushless);
    private CANSparkMax rightPrimaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.RIGHT_PRIMARY,
            MotorType.kBrushless);
    private CANSparkMax rightSecondaryMotor = new CANSparkMax(Constants.Drivetrain.CANIDs.RIGHT_SECONDARY,
            MotorType.kBrushless);

    private MotorControllerGroup leftMotors = new MotorControllerGroup(leftPrimaryMotor, leftSecondaryMotor);
    private MotorControllerGroup rightMotors = new MotorControllerGroup(rightPrimaryMotor, rightSecondaryMotor);

    private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    public Drivetrain() {
        leftMotors.setInverted(Constants.Drivetrain.LEFT_IS_INVERTED);
        leftMotors.setInverted(Constants.Drivetrain.RIGHT_IS_INVERTED);
    }

}

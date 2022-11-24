package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Robot extends TimedRobot {
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

    private XboxController controller = new XboxController(0);

    @Override
    public void robotInit() {
        leftMotors.setInverted(Constants.Drivetrain.LEFT_IS_INVERTED);
        rightMotors.setInverted(Constants.Drivetrain.RIGHT_IS_INVERTED);
    }

    @Override
    public void teleopPeriodic() {
        if (Constants.Teleop.IS_TANK_DRIVE)
            drive.tankDrive(-controller.getLeftY(), -controller.getRightY());
        else
            drive.arcadeDrive(-controller.getLeftY(), controller.getRightX());
    }
}

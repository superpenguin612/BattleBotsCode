package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Robot extends TimedRobot {
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
     * The XboxController object will allow you to read joystick inputs to then feed
     * to the motors. Keep in mind that pushing forward on the joysticks results in
     * a negative value, which is why we negate the inputs later on.
     */
    private XboxController controller = new XboxController(0);

    @Override
    public void robotInit() {
        // Your one side of your drivetrain will need to be inverted such that positive voltages result in going forward. 
        // If your robot is reversed, flip which motor is set to inverted.
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoMain2903;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Spin2903;
import frc.robot.subsystems.ArduinoLidar2903;
import frc.robot.subsystems.Climb2903;
import frc.robot.subsystems.ColorWheel2903;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight2903;
import frc.robot.subsystems.NavX2903;
import frc.robot.subsystems.Shooter2903;
import frc.robot.subsystems.SwerveDrive2903;
import frc.robot.subsystems.LIDAR_Lite2903;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    public final SwerveDrive2903 swerveDriveSubsystem = new SwerveDrive2903();
    public final Climb2903 climbSubsystem = new Climb2903();
    public final Shooter2903 shooterSubsystem = new Shooter2903();
    public final AHRS ahrs = new AHRS();
    public final NavX2903 navXSubsystem = new NavX2903();
    public final Joystick driveJoy = new Joystick(RobotMap.driveJoy);
    public final Joystick opJoy = new Joystick(RobotMap.opJoy);
    public final Limelight2903 limelightSubsystem = new Limelight2903();
    public final ArduinoLidar2903 lidarSubsystem = new ArduinoLidar2903();
    public final ColorWheel2903 colorWheelSubsystem = new ColorWheel2903();
    public final LIDAR_Lite2903 LIDAR_Lite2903 = new LIDAR_Lite2903(new DigitalInput(0));
    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
    public static NetworkTableInstance ntinst;
    public static NetworkTable tensorTable;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    //public PIDController swervePower = new PIDController(0.1, 0, 0);
    //public PIDController swerveAngle = new PIDController(0.1, 0, 0);
    public PIDController visionStrafe = new PIDController(0.1, 0, 0);
    public PIDController visionTurn = new PIDController(0.1, 0, 0);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        navXSubsystem.zero();
        ntinst = NetworkTableInstance.getDefault();  
        tensorTable = ntinst.getTable("tensorflow");   
        configureButtonBindings();
        m_chooser.setDefaultOption("Main Auto", new AutoMain2903());
        m_chooser.addOption("Spiiiiinnnnn", new Spin2903());
        SmartDashboard.putData("Auto mode", m_chooser);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_chooser.getSelected();
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter2903 extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    final double POWER_CELL_WEIGHT = 0.142; // Kg
    final double POWER_CELL_DIAMETER = 17.78; //cm
    final double MAX_VEL = 111.5; // m/s
    final double MAX_ANGLE = 45; // degree
    final double GRAV = 9.80665; // m/s/s

    WPI_TalonSRX shooterWheelL;
    WPI_TalonSRX shooterWheelR;
    WPI_TalonSRX shooterAngle;
    WPI_TalonSRX intake;

    public Shooter2903() {
        shooterWheelL = new WPI_TalonSRX(RobotMap.shooterWheelL);
        shooterWheelR = new WPI_TalonSRX(RobotMap.shooterWheelR);
        shooterAngle = new WPI_TalonSRX(RobotMap.shooterAngle);
        intake = new WPI_TalonSRX(RobotMap.intake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void shoot(double vel) {
        double power = vel / MAX_VEL; //calc power
        shooterWheelL.set(power);
        shooterWheelR.set(power);
    }

    public void setAngle(double angle, double power) {
        shooterAngle.set(power);
    }

    public void intake(double power) {
        intake.set(power);
    }

    public double shooterAngleMath(double distance, double vel) {
        double angle = 0;
        double mult = 0.95;
        // math formula without air ressitance
        angle = (Math.asin((distance * GRAV) / Math.pow(vel, 2))) / 2;
        // add multiplier
        angle = angle * mult;
        if (angle < MAX_ANGLE)
            return angle; // looking if angle is lower max angle
        else
            return -1; // -1 stands for error
    }

    public double shooterVelMath(double distance, double angle) {
        double vel = 0;
        double mult = 0.95;
        // math formula without air ressitance
        vel = Math.sqrt((distance * GRAV) / Math.sin(2 * angle));
        // add multiplier
        vel = vel * mult;
        if (vel < MAX_VEL)
            return vel; // looking if velocity is lower max velocity
        else
            return -1; // -1 stands for error
    }

    class ShooterPID extends PIDSubsystem {

        private final WPI_TalonSRX motor;
        private final SimpleMotorFeedforward m_shooterFeedforward;

        public ShooterPID(PIDController controller, double tolerance, WPI_TalonSRX motor, double kSVolts, double kVVoltSecondsPerRotation) {
            super(controller);
            getController().setTolerance(tolerance);
            setSetpoint(0);
            this.motor = motor;
            m_shooterFeedforward = new SimpleMotorFeedforward(kSVolts, kVVoltSecondsPerRotation);
        }

        @Override
        protected void useOutput(double output, double setpoint) {
            motor.set(ControlMode.PercentOutput, output+m_shooterFeedforward.calculate(setpoint));
        }

        @Override
        protected double getMeasurement() {
            return 0;
        }
        
    }
}
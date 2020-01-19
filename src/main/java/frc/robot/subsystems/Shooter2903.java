package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter2903 extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    final double POWER_CELL_WEIGHT = 0.142; // Kg
    final double POWER_CELL_DIAMETER = 17.78; //cm
    final double MAX_VEL = 111.5; // m/s
    final double MAX_SHOOT_ANGLE = 45; // degree
    final double GRAV = 9.80665; // m/s/s
    final double DEG_PER_REV = 360; //degrees per revolution
    final double TICKS_PER_REV = 4096; //ticks per revolution
    final double MAX_LIMIT_ANGLE = 61.25; //highest degrees possible

    WPI_TalonSRX shooterWheelL;
    WPI_TalonSRX shooterWheelR;
    WPI_TalonSRX shooterAngle;
    WPI_TalonSRX intake;
    DigitalInput shooterAngleTopLimit;
    DigitalInput shooterAngleBottomLimit;


    public Shooter2903() {
        shooterWheelL = new WPI_TalonSRX(RobotMap.shooterWheelL);
        shooterWheelR = new WPI_TalonSRX(RobotMap.shooterWheelR);
        shooterAngle = new WPI_TalonSRX(RobotMap.shooterAngle);
        intake = new WPI_TalonSRX(RobotMap.intake);
        shooterAngleTopLimit = new DigitalInput(RobotMap.shooterAngleTopLimit);
        shooterAngleBottomLimit = new DigitalInput(RobotMap.shooterAngleBottomLimit);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void zeroShooterAngle() {
        while (!shooterAngleBottomLimit.get()){
            shooterAngle.set(ControlMode.PercentOutput, -0.5);
        }
        shooterAngle.set(ControlMode.PercentOutput, 0);
        shooterAngle.setSelectedSensorPosition(0);        
    }

    public void shoot(double metersPerSec) {
        double velocity = convertToTalonVelocity(metersPerSec); //calc power
        shooterWheelL.set(ControlMode.Velocity,velocity);
        shooterWheelR.set(ControlMode.Velocity,velocity);
    }

    public void setAngle(double angle) {
        shooterAngle.set(ControlMode.Position,angle);
    }

    public void intake(double power) {
        intake.set(ControlMode.PercentOutput,power);
    }

    public double shooterAngleMath(double distance, double vel) {
        double angle = 0;
        double mult = 0.95;
        // math formula without air ressitance
        angle = (Math.asin((distance * GRAV) / Math.pow(vel, 2))) / 2;
        // add multiplier
        angle = angle * mult;
        if (angle < MAX_SHOOT_ANGLE)
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

    public int convertToTalonVelocity(double metersPerSec) {
        //distance wheel spins each revolution
        double circumference = Math.PI * Math.pow(POWER_CELL_DIAMETER/2,2);
        //ticks encoder counts per meter
        double ticksPerMeter = TICKS_PER_REV / circumference;
        //(m per sec) * (tick per m) = (tick per sec)
        double ticksPerSec = metersPerSec * ticksPerMeter;
        //(tick per sec) * (sec per 10 * tenth sec) = (tick per tenth sec)
        double ticksPerTenthSec = ticksPerSec / 10;
        return (int)ticksPerTenthSec;
    }

    public int convertAngleToTicks(double degrees) {
        double remainder = degrees;
        remainder /= DEG_PER_REV;
        return (int)(remainder * TICKS_PER_REV);
    }

}
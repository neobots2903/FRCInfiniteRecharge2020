package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter2903 extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    final double POWER_CELL_WEIGHT = 0.142; // Kg
    final double POWER_CELL_DIAMETER = 17.78; //cm
    final double ROBOT_SHOOTER_HEIGHT = 0.33782; //m
    final double GOAL_HEIGHT = 2.5 - ROBOT_SHOOTER_HEIGHT; //m
    final double MAX_VEL = 50; // m/s
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

    public void shootSpeed(double metersPerSec) {
        double velocity = convertToTalonVelocity(metersPerSec); //calc power
        shooterWheelL.set(ControlMode.Velocity,velocity);
        shooterWheelR.set(ControlMode.Velocity,velocity);
    }
    public void shooting(double distance, double timeCorrect){
        double[] data = shootMath(distance, timeCorrect);
        if(data[0] == -1) SmartDashboard.putString("Error:", "velocity is to big");
        if(data[1] == -1)SmartDashboard.putString("Error:", "angle is to big");
        if(data[0] >= 0 && data[1] >= 0){
            shootSpeed(data[0]);
            setAngle(data[1]);
        }
        
    }

    public void setAngle(double angle) {
        double ticks = convertAngleToTicks(angle);
        shooterAngle.set(ControlMode.Position,ticks);
    }

    public void intake(double power) {
        intake.set(ControlMode.PercentOutput,power);
    }

    public double tMax(double timeCorrect){
        double time = Math.sqrt((2*GOAL_HEIGHT)/GRAV);
        return time*timeCorrect;
    }

    public double VelInitX(double distance, double timeCorrect){
        double Vix = distance/tMax(timeCorrect);
        return Vix;
    }

    public double VelInitY(double timeCorrect){
        double Viy = (GOAL_HEIGHT + (0.5*GRAV*(Math.pow (tMax(timeCorrect), 2))))/tMax(timeCorrect);
        return Viy;
    }

    public double InitVel(double distance, double timeCorrect){
        double Vi = Math.sqrt(Math.pow(VelInitX(distance, timeCorrect), 2) + Math.pow(VelInitY(timeCorrect), 2));
        return Vi;
    }

    public double Angle(double distance, double timeCorrect){
        double angle = Math.atan(VelInitY(timeCorrect)/VelInitX(distance, timeCorrect))*(180/Math.PI);

        return angle;
    }

    public double[] shootMath(double distance, double timeCorrect){
        double InitVel = InitVel(distance, timeCorrect);
        double Angle = Angle(distance, timeCorrect);
        for(double a = 1.5; Angle > MAX_SHOOT_ANGLE && InitVel < MAX_VEL+5; a+=0.25){
            InitVel = InitVel(distance, timeCorrect/a); //dividing by a makes bigger velocity and smaller angle 
            Angle = Angle(distance, timeCorrect/a);
        }
        double[] data = {InitVel, Angle};
        if(InitVel > MAX_VEL && InitVel < MAX_VEL+5) data[0] = MAX_VEL;
        if(InitVel > MAX_VEL+5) data[0] = -1;
        if(Angle > MAX_SHOOT_ANGLE) data[1] = -1;
        return data;
    }

    // public double shooterAngleMath(double distance, double vel) {
    //     double angle = 0;
    //     double mult = 0.95;
    //     angle = (Math.asin((distance * GRAV) / Math.pow(vel, 2))) / 2;// math formula without air ressitance
    //     // angle = angle * mult;// add multiplier
    //     if (angle < MAX_SHOOT_ANGLE)
    //         return angle; // looking if angle is lower than max angle
    //     else
    //         return -1; // -1 stands for error
    // }

    // public double shooterVelMath(double distance, double angle) {
    //     double vel = 0;
    //     double mult = 0.95;
    //     // math formula without air ressitance
    //     vel = Math.sqrt((distance * GRAV) / Math.sin(2 * angle));
    //     // add multiplier
    //     // vel = vel * mult;
    //     if (vel < MAX_VEL)
    //         return vel; // looking if velocity is lower max velocity
    //     else
    //         return -1; // -1 stands for error
    // }

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
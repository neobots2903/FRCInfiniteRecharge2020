package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter2903 extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    final double POWER_CELL_WEIGHT = 0.142; // Kg
    final double POWER_CELL_DIAMETER = 17.78; // cm
    final double WHEEL_DIAMETER = 10.16; //cm
    final double ROBOT_SHOOTER_HEIGHT = 0.33782; // m
    final double GOAL_HEIGHT = 2.5 - ROBOT_SHOOTER_HEIGHT; // m
    final double MAX_VEL = 50; // m/s
    public final double MAX_SHOOT_ANGLE = 45; // degree
    final double GRAV = 9.80665; // m/s/s
    final double DEG_PER_REV = 360; // degrees per revolution
    final double TICKS_PER_REV = 4096; // ticks per revolution
    final double MAX_LIMIT_ANGLE = 57; // highest degrees possible
    final double PORT_DEPTH = 0.74295; // m
    final double SPEED_ERROR = 1.5; // m/s

    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    double lastSetSpeed = 0;

    WPI_TalonSRX shooterWheelL;
    WPI_TalonSRX shooterWheelR;
    WPI_TalonSRX shooterAngle;
    WPI_TalonSRX intake;
    Servo intakeDropper;
    DigitalInput shooterAngleTopLimit;
    DigitalInput shooterAngleBottomLimit;
    AnalogInput intakeDetect;

    public Shooter2903() {
        shooterWheelL = new WPI_TalonSRX(RobotMap.shooterWheelL);
        shooterWheelR = new WPI_TalonSRX(RobotMap.shooterWheelR);
        shooterAngle = new WPI_TalonSRX(RobotMap.shooterAngle);
        intake = new WPI_TalonSRX(RobotMap.intake);
        intakeDropper = new Servo(RobotMap.intakeDropper);
        // shooterAngleTopLimit = new DigitalInput(RobotMap.shooterAngleTopLimit);
        // shooterAngleBottomLimit = new DigitalInput(RobotMap.shooterAngleBottomLimit);
        // intakeDetect = new AnalogInput(RobotMap.intakeDetect);
        shooterWheelL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        shooterWheelR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        shooterAngle.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, kPIDLoopIdx, kTimeoutMs);
        shooterWheelL.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        shooterWheelR.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        shooterWheelL.config_kP(0, Double.MAX_VALUE);
        shooterWheelR.config_kP(0, Double.MAX_VALUE);
        // shooterWheelL.configPeakOutputForward(1);
        // shooterWheelR.configPeakOutputForward(0);
        // shooterWheelL.configPeakOutputReverse(0);
        // shooterWheelR.configPeakOutputReverse(1);
        shooterAngle.config_kP(0, 4.5);
        shooterAngle.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intakeOpen() {
        intakeDropper.set(1);
    }

    public void intakeClose() {
        intakeDropper.set(0);
    }

    public boolean intakeDetect(){
        if((intakeDetect.getVoltage()/2)/2.54 < 8) return true; else return false;
    }

    public void zeroShooterAngle() {
        while (!shooterAngleBottomLimit.get()) {
            shooterAngle.set(ControlMode.PercentOutput, -0.5);
        }
        shooterAngle.set(ControlMode.PercentOutput, 0);
        shooterAngle.setSelectedSensorPosition(0);
    }

    public void shootSpeed(double metersPerSec) {
        lastSetSpeed = metersPerSec;
        double velocity = convertToTalonVelocity(metersPerSec); // calc power
        SmartDashboard.putNumber("Target shoot speed", metersPerSec);
        shooterWheelL.set(ControlMode.Velocity, velocity);
        shooterWheelR.set(ControlMode.Velocity, -velocity);
    }

    public void stopShoot() {
        shooterWheelL.set(ControlMode.PercentOutput, 0);
        shooterWheelR.set(ControlMode.PercentOutput, 0);
    }

    public void waitForTargetSpeed() {
        while (Math.abs(lastSetSpeed - getCurrentSpeed()) > SPEED_ERROR) {
            try {
                wait(100);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public double getLeftSpeed() {
        return convertToMetersPerSec(shooterWheelL.getSelectedSensorVelocity());
    }

    public double getRightSpeed() {
        return -convertToMetersPerSec(shooterWheelR.getSelectedSensorVelocity());
    }

    public double getCurrentSpeed(){
       return (getLeftSpeed() + getRightSpeed())/2;
    }

    /**
     * Configures shooter to correct angle and speed with distance in m
     * @param distance distance to the ports in m
     * @param timeCorrect standard is 1 || decrease value for faster shoot || it decreases 
     * time automatically when shoot is not possible with current time
     */
    public void shooting(double distance, double timeCorrect){
        double[] data = shootMath(distance, timeCorrect);
        if(data[0] == -1) SmartDashboard.putString("ShootError:", "velocity is too big");
        if(data[1] == -1)SmartDashboard.putString("ShootError:", "angle is too big");
        if(data[0] >= 0 && data[1] >= 0){
            SmartDashboard.putString("ShootError:", "none");
            shootSpeed(data[0]);
            setAngle(data[1]);
        }
        
    }

    public void setAngle(double angle) {
        if (angle < 0) angle = 0;
        if (angle > MAX_SHOOT_ANGLE) angle = MAX_SHOOT_ANGLE;
        SmartDashboard.putNumber("Target shoot angle", angle);
        double ticks = convertAngleToTicks(angle);
        shooterAngle.set(ControlMode.Position,ticks);
        SmartDashboard.putNumber("Target shoot angle ticks", ticks);
    }

    public double getAngle(){
        return convertTicksToAngle(getAngleTicks());
    }

    public int getAngleTicks() {
        return shooterAngle.getSelectedSensorPosition();
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
        distance += PORT_DEPTH;
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
        double circumference = Math.PI * WHEEL_DIAMETER/100;
        //ticks encoder counts per meter
        double ticksPerMeter = TICKS_PER_REV / circumference;
        //(m per sec) * (tick per m) = (tick per sec)
        double ticksPerSec = metersPerSec * ticksPerMeter;
        //(tick per sec) * (sec per 10 * tenth sec) = (tick per tenth sec)
        double ticksPerTenthSec = ticksPerSec / 10;
        return (int)ticksPerTenthSec;
    }

    public double convertToMetersPerSec(double talonVelocity) {
        double ticksPerTenthSec = talonVelocity;
        //(ticks per tenth sec) * 10 = (tick per sec)
        double ticksPerSec = ticksPerTenthSec*10;
        //distance wheel spins each revolution
        double circumference = Math.PI * WHEEL_DIAMETER/100;
        //meters per ticks encoder count
        double metersPerTick = circumference / TICKS_PER_REV;
        //(ticks per sec) * (meters per tick) = (meters per sec)
        double metersPerSecond = ticksPerSec * metersPerTick;
        return metersPerSecond;
    }

    public int convertAngleToTicks(double degrees) {
        double remainder = degrees;
        remainder /= DEG_PER_REV;
        return (int)(remainder * TICKS_PER_REV);
    }

    public int convertTicksToAngle(double ticks) {
        double remainder = ticks;
        remainder /= TICKS_PER_REV;
        return (int)(remainder * DEG_PER_REV);
    }

}
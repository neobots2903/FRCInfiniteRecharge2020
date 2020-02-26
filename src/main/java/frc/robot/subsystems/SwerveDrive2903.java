package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwerveDrive2903 extends SubsystemBase {

    public SwerveModule2903 LeftFront;
    public SwerveModule2903 RightFront;
    public SwerveModule2903 RightRear;
    public SwerveModule2903 LeftRear;

    public List<SwerveModule2903> modules = new ArrayList<SwerveModule2903>();

    final int TICKS_PER_REV = 4096 * 6;
    final int DEG_PER_REV = 360;
    boolean isForward = true;

    double joyDeadzone = 0.5; // joystick isn't actually in center, making sure doesn't move when not touched
                              // :)
    double triggerDeadzone = 0.05;
    int targetAngle = 0;

    public SwerveDrive2903() {
        LeftFront = new SwerveModule2903(RobotMap.LeftFrontForward, RobotMap.LeftFrontTurn, RobotMap.LeftFrontLimit);
        RightFront = new SwerveModule2903(RobotMap.RightFrontForward, RobotMap.RightFrontTurn,
                RobotMap.RightFrontLimit);
        RightRear = new SwerveModule2903(RobotMap.RightRearForward, RobotMap.RightRearTurn, RobotMap.RightRearLimit);
        LeftRear = new SwerveModule2903(RobotMap.LeftRearForward, RobotMap.LeftRearTurn, RobotMap.LeftRearLimit);

        LeftFront.setTurnDegreeOffset(225);
        RightFront.setTurnDegreeOffset(315);
        RightRear.setTurnDegreeOffset(45);
        LeftRear.setTurnDegreeOffset(135);

        modules.add(LeftFront);
        modules.add(RightFront);
        modules.add(RightRear);
        modules.add(LeftRear);
    }

    public void zeroModulesLimit() {
        ArrayList<Thread> threads = new ArrayList<Thread>();
        for (SwerveModule2903 module : modules)
            threads.add(new Thread(() -> {
                module.zeroTurnMotor();
            }));
        for (Thread thread : threads)
            thread.start();
        for (Thread thread : threads)
            try {
                thread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
    }

    public void zeroModules() {
        for (SwerveModule2903 module : modules)
            module.setZero();
    }

    public void goToZero() {
        setTurnDegrees(0);
    }

    public int joystickAngle(double x, double y) {
        int angle = -1;
        if (Math.abs(x) > joyDeadzone || Math.abs(y) > joyDeadzone) {
            angle = (int)Math.toDegrees(Math.atan2(x, y));
            if (angle < 0)
                angle += 360;
        }
        return angle;
    }

    public double joystickMag(double x, double y) {
        return Math.sqrt(x*x+y*y);
    }

    public void setForward(double speed) {
        for (SwerveModule2903 module : modules)
            module.ForwardMotor.set(speed * (isForward ? 1 : -1));
    }

    public void setTurnDegrees(int degrees) {
        if (degrees == -1) return;
        int currentTargetTic = (int)LeftFront.TurnMotor.getClosedLoopTarget()-LeftFront.getLastJoyTurnTicks();
        int localTic = LeftFront.angleToTicks(degrees) - currentTargetTic % TICKS_PER_REV;
        if (localTic < -TICKS_PER_REV/2)
            localTic += TICKS_PER_REV;
        else if (localTic > TICKS_PER_REV/2)
            localTic -= TICKS_PER_REV;
        if (Math.abs(localTic) > TICKS_PER_REV/4) {
            isForward = !isForward;
            for (SwerveModule2903 module : modules) {
                module.setEncoder(currentTargetTic + TICKS_PER_REV/2*(isForward ? 1 : -1));
            module.TurnMotor.set(ControlMode.Position,currentTargetTic + TICKS_PER_REV/2*(isForward ? 1 : -1));
            }
            setTurnDegrees(degrees);
            return;
        }
        for (SwerveModule2903 module : modules)
            module.TurnMotor.set(ControlMode.Position,currentTargetTic+localTic+module.getJoyTurnTicks(degrees));
    }

    public void swerveDrive(double power, double angle, double turn, boolean fieldCentric) {

        for (SwerveModule2903 module : modules)
            module.setJoyTurnPercent(turn);
        
        if (angle != -1) 
            targetAngle = (int)angle;
        setTurnDegrees((int)(targetAngle-((fieldCentric)?Robot.robotContainer.navXSubsystem.turnAngle():0)));

        if (triggerDeadzone < Math.abs(power)) {
            setForward(power);
        }
    }
    
    public void swerveDriveDistance(double power, double angle, boolean fieldCentric, double distance){
        LeftFront.zeroForwardEncoder();
        RightRear.zeroForwardEncoder();
        while((LeftFront.getForwardMeters() + RightRear.getForwardMeters())/2 < distance){
            swerveDrive(power, angle, 0, fieldCentric);
        }
        swerveDrive(0, angle, 0, fieldCentric);
    }

    public void TankDrive(double left, double right) {
        goToZero();
        LeftFront.ForwardMotor.set(left * (isForward ? 1 : -1));
        LeftRear.ForwardMotor.set(left * (isForward ? 1 : -1));
        RightRear.ForwardMotor.set(right * (isForward ? 1 : -1));
        RightFront.ForwardMotor.set(right * (isForward ? 1 : -1));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
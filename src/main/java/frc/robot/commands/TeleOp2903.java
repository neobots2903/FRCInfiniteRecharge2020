package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;

public class TeleOp2903 extends CommandBase {
    final RobotContainer r;
    private boolean zeroLock = false;
    private boolean oneLock = false;
    private boolean autoAim = false;
    private boolean fieldCentric = false;
    private boolean climbActive = false;
    private boolean climbLock = false;
    private boolean climbRaised = false;
    private boolean climbExtend = false;
    
    
    public TeleOp2903(RobotContainer robot) {
        r = robot;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        r.limelightSubsystem.setLight(false);
        r.limelightSubsystem.setTargetMode();
        r.swerveDriveSubsystem.zeroModulesLimit();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double y = -r.driveJoy.getRawAxis(1);
        double x = r.driveJoy.getRawAxis(0);
        // double forward = r.driveJoy.getRawAxis(2);
        // double backward = r.driveJoy.getRawAxis(3);
        double forward = 0;
        double backward = r.swerveDriveSubsystem.joystickMag(x, y);
        double turnX = r.driveJoy.getRawAxis(4);
        if(Math.abs(turnX) < 0.01)turnX = 0;

        if(r.driveJoy.getRawButton(8)){
            if(!zeroLock){
                fieldCentric = !fieldCentric;
                zeroLock = true;
            }else{
                zeroLock = false;
            }
        }

        SmartDashboard.putNumber("LF Amp", r.swerveDriveSubsystem.LeftFront.TurnMotor.getStatorCurrent());
        SmartDashboard.putNumber("LR Amp", r.swerveDriveSubsystem.LeftRear.TurnMotor.getStatorCurrent());
        SmartDashboard.putNumber("RF Amp", r.swerveDriveSubsystem.RightFront.TurnMotor.getStatorCurrent());
        SmartDashboard.putNumber("RR Amp", r.swerveDriveSubsystem.RightRear.TurnMotor.getStatorCurrent());

        SmartDashboard.putBoolean("LF on zero?", r.swerveDriveSubsystem.LeftFront.getLimit());
        SmartDashboard.putBoolean("LR on zero?", r.swerveDriveSubsystem.LeftRear.getLimit());
        SmartDashboard.putBoolean("RF on zero?", r.swerveDriveSubsystem.RightFront.getLimit());
        SmartDashboard.putBoolean("RR on zero?", r.swerveDriveSubsystem.RightRear.getLimit());

        SmartDashboard.putNumber("LF Deg", r.swerveDriveSubsystem.LeftFront.getAbsoluteTurnDegrees());
        SmartDashboard.putNumber("LR Deg", r.swerveDriveSubsystem.LeftRear.getAbsoluteTurnDegrees());
        SmartDashboard.putNumber("RF Deg", r.swerveDriveSubsystem.RightFront.getAbsoluteTurnDegrees());
        SmartDashboard.putNumber("RR Deg", r.swerveDriveSubsystem.RightRear.getAbsoluteTurnDegrees());

        SmartDashboard.putNumber("LF FW M", r.swerveDriveSubsystem.LeftFront.getForwardMeters());
        SmartDashboard.putNumber("LR FW M", r.swerveDriveSubsystem.LeftRear.getForwardMeters());
        SmartDashboard.putNumber("RF FW M", r.swerveDriveSubsystem.RightFront.getForwardMeters());
        SmartDashboard.putNumber("RR FW M", r.swerveDriveSubsystem.RightRear.getForwardMeters());

        SmartDashboard.putNumber("Shooter Angle", r.shooterSubsystem.getAngle());
        SmartDashboard.putNumber("Shooter Angle Ticks", r.shooterSubsystem.getAngleTicks());
        SmartDashboard.putNumber("Shooter Speed", r.shooterSubsystem.getCurrentSpeed());
        SmartDashboard.putNumber("Shooter Speed Left", r.shooterSubsystem.getLeftSpeed());
        SmartDashboard.putNumber("Shooter Speed Right", r.shooterSubsystem.getRightSpeed());

        SmartDashboard.putNumber("Lidar Distance", r.LIDAR_Lite2903.getDistance());
        SmartDashboard.putNumber("Gyro Angle", r.navXSubsystem.turnAngle());

        SmartDashboard.putBoolean("Auto aim?", autoAim);
        SmartDashboard.putString("Climber State:", "Unknown sorry");

        if (autoAim && r.limelightSubsystem.getTV() == 1) {
            r.limelightSubsystem.setLight(true);
            turnX = MathUtil.clamp(r.visionTurn.calculate(r.limelightSubsystem.getTX(), 0),-1,1);
        } else {
            r.limelightSubsystem.setLight(false);
        }

        r.swerveDriveSubsystem.swerveDrive(forward-backward, r.swerveDriveSubsystem.joystickAngle(x, y), turnX, fieldCentric);

        if(r.opJoy.getRawButton(1)){
            double distance = r.LIDAR_Lite2903.getDistance(); //lidar distance 
            r.shooterSubsystem.shooting(distance, 1);
        } else {
            r.shooterSubsystem.stopShoot();
        }

        double intakePower = -r.opJoy.getRawAxis(5);
        r.shooterSubsystem.intake(intakePower);

        if(r.opJoy.getRawButton(8)){
            if(!climbLock){
                climbLock = true;
                if(climbActive == true)climbActive = false;else climbActive = true;
            }

        }else{
            climbLock = false;
        }

        if(r.opJoy.getRawButton(3) && climbActive){
            if(!climbRaised){
                r.climbSubsystem.RaiseArms();
                climbRaised = true;
            }
        }

        if(r.opJoy.getPOV()==0 && climbActive){
            if(!climbExtend && climbRaised){
                r.climbSubsystem.ExtendArm();
                climbExtend = true;
            }
        }

        if(r.opJoy.getPOV()==180 && climbActive){
            if(climbExtend){
                r.climbSubsystem.LowerArms();
            }
        }

        // if(r.opJoy.getRawButton(2)){
        //     r.colorWheelSubsystem.spin(3);
        // }
        // if(r.opJoy.getRawButton(4)){
        //     r.colorWheelSubsystem.spinToColor(0.75);
        // }

        if(r.opJoy.getRawButton(2)){
            if(!oneLock){
                autoAim = !autoAim;
                oneLock = true;
            }else{
                oneLock = false;
            }
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
  }
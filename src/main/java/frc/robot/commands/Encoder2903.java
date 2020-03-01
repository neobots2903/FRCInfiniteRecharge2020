
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule2903;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Encoder2903 extends CommandBase {
    final RobotContainer r;
    
    public Encoder2903(RobotContainer r) {
        this.r = r;
    }

    @Override
    public void initialize() {
        r.swerveDriveSubsystem.stopDrive();
    }


    @Override
    public void execute() {
        SmartDashboard.putNumber("LF Deg", r.swerveDriveSubsystem.LeftFront.getAbsoluteTurnDegrees());
        SmartDashboard.putNumber("LR Deg", r.swerveDriveSubsystem.LeftRear.getAbsoluteTurnDegrees());
        SmartDashboard.putNumber("RF Deg", r.swerveDriveSubsystem.RightFront.getAbsoluteTurnDegrees());
        SmartDashboard.putNumber("RR Deg", r.swerveDriveSubsystem.RightRear.getAbsoluteTurnDegrees());

        SmartDashboard.putBoolean("LF on zero?", r.swerveDriveSubsystem.LeftFront.getLimit());
        SmartDashboard.putBoolean("LR on zero?", r.swerveDriveSubsystem.LeftRear.getLimit());
        SmartDashboard.putBoolean("RF on zero?", r.swerveDriveSubsystem.RightFront.getLimit());
        SmartDashboard.putBoolean("RR on zero?", r.swerveDriveSubsystem.RightRear.getLimit());

        SmartDashboard.putNumber("LF FW M", r.swerveDriveSubsystem.LeftFront.getForwardMeters());
        SmartDashboard.putNumber("LR FW M", r.swerveDriveSubsystem.LeftRear.getForwardMeters());
        SmartDashboard.putNumber("RF FW M", r.swerveDriveSubsystem.RightFront.getForwardMeters());
        SmartDashboard.putNumber("RR FW M", r.swerveDriveSubsystem.RightRear.getForwardMeters());

        SmartDashboard.putNumber("Shooter Angle", r.shooterSubsystem.getAngle());
        SmartDashboard.putNumber("Shooter Speed", r.shooterSubsystem.getCurrentSpeed());
        SmartDashboard.putNumber("Shooter Speed Left", r.shooterSubsystem.getLeftSpeed());
        SmartDashboard.putNumber("Shooter Speed Right", r.shooterSubsystem.getRightSpeed());

        SmartDashboard.putNumber("Lidar Distance", r.LIDAR_Lite2903.getDistance());
        SmartDashboard.putNumber("Gyro Angle", r.navXSubsystem.turnAngle());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

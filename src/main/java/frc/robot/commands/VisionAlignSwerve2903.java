/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArduinoLidar2903.LidarPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class VisionAlignSwerve2903 extends CommandBase {

    double maxTA = 3.4;

    double forward;
    double side;
    double turn;

    double power;
    double angle;

    public VisionAlignSwerve2903() {
        
    }

    @Override
    public void initialize() {
        Robot.robotContainer.limelightSubsystem.setTargetMode();
    }

    @Override
    public void execute() {

        double ta = Robot.robotContainer.limelightSubsystem.getTA();
        int leftLid = Robot.robotContainer.lidarSubsystem.getDistance(LidarPosition.LEFT);
        int rightLid = Robot.robotContainer.lidarSubsystem.getDistance(LidarPosition.RIGHT);

        if (Robot.robotContainer.limelightSubsystem.getTV() == 1) {
            forward = percentToTarget(ta, maxTA)*0.4;
            side = Robot.robotContainer.visionStrafe.calculate(Robot.robotContainer.limelightSubsystem.getTX(), 0)*0.75;
            turn = -Robot.robotContainer.visionTurn.calculate(Robot.robotContainer.limelightSubsystem.getTS(), 0);
        } else {
            forward = 0;
            side = 0;
            turn = 0;
        }

        if (Robot.robotContainer.lidarSubsystem.getStatus(LidarPosition.LEFT) == 0 &&
            Robot.robotContainer.lidarSubsystem.getStatus(LidarPosition.RIGHT) == 0) {
            
            int turnError = leftLid - rightLid;

            if (Math.abs(turnError) > 25) 
                turn = turnError;
            
            turn/=100;

        }

        Robot.robotContainer.swerveDriveSubsystem.swerveDrive(Math.sqrt(Math.pow(forward, 2)+Math.pow(side, 2)), 
            Math.atan2(forward, side), 0, false);

    }
    
    double percentToTarget(double value, double target) {
        double sign = (value < 0) ? -1 : 1;
        return ((target - Math.abs(value)) / target) * sign;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

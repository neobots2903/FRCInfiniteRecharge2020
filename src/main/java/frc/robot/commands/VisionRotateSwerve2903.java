/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class VisionRotateSwerve2903 extends CommandBase {

    double turnPower;
    double turnDirection;

    final double ERROR = 5;

    public VisionRotateSwerve2903() {
        
    }

    @Override
    public void initialize() {
        Robot.robotContainer.limelightSubsystem.setTargetMode();
    }

    @Override
    public void execute() {

        if (Robot.robotContainer.limelightSubsystem.getTV() != 1)
            turnPower = 0;
        else
            turnPower = Robot.robotContainer.visionStrafe.calculate(Robot.robotContainer.limelightSubsystem.getTX(), 0)*0.75;
        
        if (Robot.robotContainer.limelightSubsystem.getTX() > ERROR) {
            turnDirection = 1;
        } else if (Robot.robotContainer.limelightSubsystem.getTX() < -ERROR) {
            turnDirection = -1;
        }

        Robot.robotContainer.swerveDriveSubsystem.swerveDrive(turnPower, 0, turnDirection, false);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

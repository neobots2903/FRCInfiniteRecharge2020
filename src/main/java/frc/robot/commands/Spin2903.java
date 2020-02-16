
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule2903;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Spin2903 extends CommandBase {
    final RobotContainer r = Robot.robotContainer;
    public Spin2903() {
        
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        for(SwerveModule2903 module : r.swerveDriveSubsystem.modules){
            module.TurnMotor.set(ControlMode.PercentOutput, 1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        for(SwerveModule2903 module : r.swerveDriveSubsystem.modules){
            module.TurnMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

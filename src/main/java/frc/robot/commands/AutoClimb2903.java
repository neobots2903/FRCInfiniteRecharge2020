package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb2903;
import frc.robot.subsystems.Limelight2903;
import frc.robot.subsystems.Shooter2903;
import frc.robot.subsystems.SwerveDrive2903;

public class AutoClimb2903 extends CommandBase{
    
    final RobotContainer r = Robot.robotContainer;
    boolean isFinished = false;
    private final double MAX_SHOOT_ANGLE = 45;
    //private final SetAngle setAngle;  
    public AutoClimb2903() {}
      
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // //Aim limelight up(attached to shooter)

        // //Line up with climb bar and go forward until we are under it, UNIFINISHED
        
        // //Speen around, locate the climb thing, go forward until we're under the bar?

        // //angle slightly crooked with one arm on each side(Turn by DEGREES_TO_TURN)

        //swerveDrive can not turn to a specific number of degrees.
        //if you would like this functionality, 

        r.shooterSubsystem.setAngle(r.shooterSubsystem.MAX_SHOOT_ANGLE);       //we would need to make a gyroTurn function.

        Robot.robotContainer.limelightSubsystem.setTargetMode();
        if(r.limelightSubsystem.getTV()==0);
 
        //turn the robot
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(r.limelightSubsystem.getTVERT() == 0); 
        r.shooterSubsystem.setAngle(0); // These are placeholder values
        r.climbSubsystem.RaiseArms();// do this first 
        r.climbSubsystem.ExtendArm();
        
        r.climbSubsystem.RetractArm();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
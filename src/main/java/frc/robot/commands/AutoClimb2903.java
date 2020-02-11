package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb2903;
import frc.robot.subsystems.Limelight2903;
import frc.robot.subsystems.Shooter2903;
import frc.robot.subsystems.SwerveDrive2903;

public class AutoClimb2903 extends CommandBase{
    
    private final Climb2903 climb2903;
    private final Shooter2903 shooter2903;
    private final Limelight2903 limelight2903;
    private final SwerveDrive2903 swerveDrive2903;
    
    public AutoClimb2903(Climb2903 climb, Shooter2903 shooter, Limelight2903 limelight, SwerveDrive2903 swerveDrive) {
        climb2903 = climb;
        shooter2903 = shooter;
        limelight2903 = limelight;
        swerveDrive2903 = swerveDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // //Aim limelight up(attached to shooter)
        // setAngle(MAX_SHOOT_ANGLE); //45 degree angle
        // //Line up with climb bar and go forward until we are under it, UNIFINISHED
        
        // //Speen around, locate the climb thing, go forward until we're under the bar?

        // //angle slightly crooked with one arm on each side(Turn by DEGREES_TO_TURN)

        // swerveDrive(50, 0, DEGREES_TO_TURN, false); //swerveDrive can not turn to a specific number of degrees.
        //                                             //if you would like this functionality, 
        //                                             //we would need to make a gyroTurn function.
        // climb2903.RaiseArms();
        // climb2903.ExtendArm();
        // //angle parallel with the bar to click in(Turn by -DEGREES_TO_TURN)
        // swerveDrive(50, 0, -DEGREES_TO_TURN, false);
        // climb2903.RetractArm();



        //Aim limelight up
        shooter2903.setAngle(shooter2903.MAX_SHOOT_ANGLE); //45 degree angle

        //Line up with climbing bar using limelight, UNFINISHED

        //Go forward 63 inches(assuming the limelight is at a 45 degrees, this is how much we have to go forward in a perfect world, but we'll have to change this later because the limelight isn't even with the ground)
        swerveDrive2903.swerveDrive(50, 0, 0, false); //These are placeholder values

        //turn the robot
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
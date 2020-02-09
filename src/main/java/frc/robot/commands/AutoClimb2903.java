package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb2903;

public class AutoClimb2903 extends CommandBase{
    
    private final Climb2903 climb2903;
    private final int DEGREES_TO_TURN = 15;
    
    public AutoClimb2903(Climb2903 climb) {
        climb2903 = climb;
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
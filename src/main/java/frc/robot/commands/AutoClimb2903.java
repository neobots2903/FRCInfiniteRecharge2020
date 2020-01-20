package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb2903;

public class AutoClimb2903 extends CommandBase{
    
    private final Climb2903 climb2903;

    public AutoClimb2903(Climb2903 climb) {
        climb2903 = climb;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //Aim limelight up, UNFINISHED
        //angle slightly crooked with one arm on each side, , UNFINISHED
        climb2903.RaiseArms();
        climb2903.ExtendArmExtend();
        //angle parallel with the bar to click in, , UNFINISHED
        climb2903.RaiseArms();
        climb2903.RetractArmExtend();
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
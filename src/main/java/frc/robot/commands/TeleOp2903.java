package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TeleOp2903 extends CommandBase {

    private boolean zeroLock = false;
    private boolean fieldCentric = false;

    public TeleOp2903() {
        
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.robotContainer.swerveDriveSubsystem.init();
        Robot.robotContainer.navXSubsystem.zero();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double y = -Robot.robotContainer.driveJoy.getRawAxis(1);
        double x = Robot.robotContainer.driveJoy.getRawAxis(0);
        double forward = Robot.robotContainer.driveJoy.getRawAxis(2);
        double backward = Robot.robotContainer.driveJoy.getRawAxis(3);
        double turnX = Robot.robotContainer.driveJoy.getRawAxis(4);
        if(Math.abs(turnX) < 0.01)turnX = 0;

        if(Robot.robotContainer.driveJoy.getRawButton(8)){
            if(!zeroLock){
                fieldCentric = !fieldCentric;
                zeroLock = true;
            }else{
                zeroLock = false;
            }
        }

        Robot.robotContainer.swerveDriveSubsystem.swerveDrive(forward-backward, Robot.robotContainer.swerveDriveSubsystem.joystickAngle(x, y), turnX, fieldCentric);
        
        if(Robot.robotContainer.opJoy.getRawButton(8)){
            //Aim limelight up(attached to shooter)
            setAngle(MAX_SHOOT_ANGLE); //45 degree angle
            //Line up with climb bar and go forward until we are under it, UNIFINISHED
            
            //Speen around, locate the climb thing, go forward until we're under the bar?

            //angle slightly crooked with one arm on each side(Turn by DEGREES_TO_TURN)
            swerveDrive(50, 0, DEGREES_TO_TURN, false);
            climb2903.RaiseArms();
            climb2903.ExtendArm();
            //angle parallel with the bar to click in(Turn by -DEGREES_TO_TURN)
            swerveDrive(50, 0, -DEGREES_TO_TURN, false);
            climb2903.RetractArm();
        }

        if(Robot.robotContainer.opJoy.getRawButton(1)){
            double distance = Robot.robotContainer.LIDAR_Lite2903.getDistance(); //lidar distance 
            Robot.robotContainer.shooterSubsystem.shooting(distance, 1);
        }

        double intakePower = -Robot.robotContainer.opJoy.getRawAxis(5);
        Robot.robotContainer.shooterSubsystem.intake(intakePower);


        if(Robot.robotContainer.opJoy.getRawButton(2)){
            Robot.robotContainer.colorWheelSubsystem.spin(3);
        }
        if(Robot.robotContainer.opJoy.getRawButton(4)){
            Robot.robotContainer.colorWheelSubsystem.spinToColor(0.75);
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
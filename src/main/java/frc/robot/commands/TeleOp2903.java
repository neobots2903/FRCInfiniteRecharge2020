package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TeleOp2903 extends CommandBase {
    final RobotContainer r = Robot.robotContainer;
    private boolean zeroLock = false;
    private boolean fieldCentric = false;
    final double MAX_SHOOT_ANGLE = r.shooterSubsystem.MAX_SHOOT_ANGLE; 
    boolean start_has_been_pressed=false;
    boolean start_held=false;
    public TeleOp2903() {
        
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        r.swerveDriveSubsystem.init();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double y = -r.driveJoy.getRawAxis(1);
        double x = r.driveJoy.getRawAxis(0);
        double forward = r.driveJoy.getRawAxis(2);
        double backward = r.driveJoy.getRawAxis(3);
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

        r.swerveDriveSubsystem.swerveDrive(forward-backward, r.swerveDriveSubsystem.joystickAngle(x, y), turnX, fieldCentric);
       
        if(Robot.robotContainer.opJoy.getRawButton(8) && !start_held ){
            //Aim limelight up(attached to shooter)
            //r.shooterSubsystem.setAngle(MAX_SHOOT_ANGLE); //45 degree angle

            start_held = true; 

            if(start_has_been_pressed){
                r.climbSubsystem.RetractArm();

            }else{
                r.climbSubsystem.RaiseArms();
                r.climbSubsystem.ExtendArm();
            }

        }else if (!r.opJoy.getRawButton(8)){
            start_held = false;
        }

        if(r.opJoy.getRawButton(1)){
            double distance = r.LIDAR_Lite2903.getDistance(); //lidar distance 
            r.shooterSubsystem.shooting(distance, 1);
        }

        double intakePower = -r.opJoy.getRawAxis(5);
        r.shooterSubsystem.intake(intakePower);


        if(r.opJoy.getRawButton(2)){
            r.colorWheelSubsystem.spin(3);
        }
        if(r.opJoy.getRawButton(4)){
            r.colorWheelSubsystem.spinToColor(0.75);
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
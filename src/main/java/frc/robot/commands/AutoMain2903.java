
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AutoMain2903 extends CommandBase {
    final double TURN_ERROR = 3; // degree
    final RobotContainer r = Robot.robotContainer;
    final double PIXEL_HEIGTH = 1080;
    final double PIXEL_WIDTH = 1920;
    final double PIXEL_ERROR = 5;
    int cellCount = 0;
   

    public AutoMain2903() {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(cellCount == 0)r.swerveDriveSubsystem.swerveDriveDistance(1, 180, false, 1.5);
        r.limelightSubsystem.setZoomMode();
        while (r.limelightSubsystem.getTV() == 0) {
            r.swerveDriveSubsystem.swerveDrive(0.5, 180, 1, false);
        }
        while (Math.abs(r.limelightSubsystem.getTX()) > TURN_ERROR / 2) {
            if (r.limelightSubsystem.getTX() > 0)
                r.swerveDriveSubsystem.swerveDrive(0.25, 180, 1, false);
            else
                r.swerveDriveSubsystem.swerveDrive(0.25, 180, -1, false);
        }

        r.shooterSubsystem.shooting(r.LIDAR_Lite2903.getDistance(), 1);

        for (int i = 0; i < 3; ++i) {
            r.shooterSubsystem.waitForTargetSpeed();
            double lastSpeed = r.shooterSubsystem.getCurrentSpeed();
            while (lastSpeed - r.shooterSubsystem.getCurrentSpeed() < 0.2) {
                lastSpeed = r.shooterSubsystem.getCurrentSpeed();
                r.shooterSubsystem.intake(1);
            }
            r.shooterSubsystem.intake(0);
        }
        
        cellCount = 0;
        boolean isCellAtSensor = false;
        while(cellCount < 3){
            if(r.tensorTable.getEntry("EnergyCellCount").getDouble(0) > 0){
                if(r.tensorTable.getEntry("EnergyCellX").getDouble(PIXEL_WIDTH/2) > (PIXEL_WIDTH/2)+PIXEL_ERROR)
                r.swerveDriveSubsystem.swerveDrive(0.5, 0, 0.5, false);
                else if(r.tensorTable.getEntry("EnergyCellX").getDouble(PIXEL_WIDTH/2) < (PIXEL_WIDTH/2)-PIXEL_ERROR)
                r.swerveDriveSubsystem.swerveDrive(0.5, 0, -0.5, false);
                
            }
            if(r.shooterSubsystem.intakeDetect())isCellAtSensor = true;
            if(!r.shooterSubsystem.intakeDetect() && isCellAtSensor){
                isCellAtSensor = false;
                cellCount++;
            }
                
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

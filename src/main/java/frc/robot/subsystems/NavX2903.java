package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * Super intense gyro thing
 */
public class NavX2903 extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean collisionDetected = false;
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  
  final static double kCollisionThreshold_DeltaG = 0.5f;

  public void zero() {
    Robot.robotContainer.ahrs.zeroYaw();
  }

  public double turnAngle() {
    return Robot.robotContainer.ahrs.getAngle();
  }

  public boolean isColliding() {
    collisionDetected = false;
    double curr_world_linear_accel_x = Robot.robotContainer.ahrs.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = Robot.robotContainer.ahrs.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
        || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
      collisionDetected = true;
    }
    return collisionDetected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

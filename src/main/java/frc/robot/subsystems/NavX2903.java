package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Super intense gyro thing
 */
public class NavX2903 extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean collisionDetected = false;
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;

  private RobotContainer r;
  
  final static double kCollisionThreshold_DeltaG = 0.5f;

  public NavX2903(RobotContainer r) {
    this.r = r;
  }

  public void zero() {
    r.ahrs.zeroYaw();
  }

  public double turnAngle() {
    return r.ahrs.getAngle();
  }

  public void setBackwards(boolean isBackwards){
    if(isBackwards)r.ahrs.setAngleAdjustment(180);
  }

  public boolean isColliding() {
    collisionDetected = false;
    double curr_world_linear_accel_x = r.ahrs.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = r.ahrs.getWorldLinearAccelY();
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

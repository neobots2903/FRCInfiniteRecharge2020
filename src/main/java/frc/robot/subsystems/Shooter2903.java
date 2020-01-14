package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter2903 extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

    final double powerCellWeight = 0.142; //Kg
    final double MAX_VEL = 111.5; // m/s
    final double MAX_ANGLE = 45; // degree
    final double GRAV = 9.8; // m/s/s

    WPI_TalonSRX shooterWheelL;
    WPI_TalonSRX shooterWheelR;

  public Shooter2903() {
    shooterWheelL = new WPI_TalonSRX(RobotMap.shooterWheelL);
    shooterWheelR = new WPI_TalonSRX(RobotMap.shooterWheelR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooting( double vel){
    double power = 0;
    power = vel/MAX_VEL;
    shooterWheelL.set(power);
  }

  public void setAngle(double angle){

  }

  public void intake(double power){

  }

  public double shooterAngleMath(double distance, double vel){
    double angle = 0;
    double mult = 0.95;
    //math formula without air ressitance
    angle = (Math.asin((distance * GRAV)/Math.pow(vel, 2)))/2;
    //add multiplier
    angle = angle * mult;
    if (angle < MAX_ANGLE)return angle; //looking if angle is lower max angle
    else return -1; //-1 stands for error
  }

  public double shooterVelMath(double distance, double angle){
    double vel = 0;
    double mult = 0.95;  
    //math formula without air ressitance
    vel = Math.sqrt((distance * GRAV)/Math.sin(2 * angle));
    //add multiplier
    vel = vel * mult;
    if (vel < MAX_VEL) return vel; //looking if velocity is lower max velocity
    else return -1; //-1 stands for error
  }
}
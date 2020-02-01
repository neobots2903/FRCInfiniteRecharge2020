package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ColorWheel2903 extends SubsystemBase {
  
    WPI_TalonSRX colorWheel;

    final double TICKS_PER_REV = 4096; //ticks per revolution
    final double COLOR_WHEEL_CIRC = 2.544690049; // m
    final double SPINNIG_WHEEL_CIRC = 0.3191858136; //m
    final double TICKS_PER_COLOR_WHEEL_REV =  Math.round(TICKS_PER_REV*(COLOR_WHEEL_CIRC/SPINNIG_WHEEL_CIRC));

  public ColorWheel2903() {
    colorWheel = new WPI_TalonSRX(RobotMap.colorWheel);
  }

  @Override
  public void periodic() {
    
  }

  public void spin(double wheelTurns){
      double ticks = TICKS_PER_COLOR_WHEEL_REV*wheelTurns;
      colorWheel.setSelectedSensorPosition(0);
      colorWheel.set(ControlMode.Position, ticks);
  }

  
}
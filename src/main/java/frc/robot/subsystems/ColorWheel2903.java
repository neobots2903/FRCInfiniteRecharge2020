package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ColorWheel2903 extends SubsystemBase {
  
    WPI_TalonSRX colorWheel;
    ColorSensorV3 colorSensor;

    final double TICKS_PER_REV = 4096; //ticks per revolution
    final double COLOR_WHEEL_CIRC = 2.544690049; // m
    final double SPINNIG_WHEEL_CIRC = 0.3191858136; //m
    final double TICKS_PER_COLOR_WHEEL_REV =  Math.round(TICKS_PER_REV*(COLOR_WHEEL_CIRC/SPINNIG_WHEEL_CIRC));

  public ColorWheel2903() {
    colorWheel = new WPI_TalonSRX(RobotMap.colorWheel);
    //colorSensor = new ColorSensorV3(Port.kMXP);
  }

  @Override
  public void periodic() {
    
  }

  public void spin(double wheelTurns){
      double ticks = TICKS_PER_COLOR_WHEEL_REV*wheelTurns;
      colorWheel.setSelectedSensorPosition(0);
      colorWheel.set(ControlMode.Position, ticks);
  }

  enum Colors {
      RED,
      GREEN,
      BLUE,
      YELLOW
  }

  public void spinToColor(double power){
      Colors fieldTargetColor = getFieldTargetColor();
      if(fieldTargetColor == null){
        SmartDashboard.putString("ColorWheelTarget:" , "No Data");
      }else{
            switch(fieldTargetColor){
                case RED:
                    while(colorSensor.getBlue() < 150){
                        colorWheel.set(ControlMode.PercentOutput, power);
                    }
                    colorWheel.set(ControlMode.PercentOutput, 0);
                    break;
                case GREEN:
                    while(colorSensor.getGreen() < 150 || colorSensor.getRed() < 150){
                        colorWheel.set(ControlMode.PercentOutput, power);
                    }
                    colorWheel.set(ControlMode.PercentOutput, 0);
                    break;
                case BLUE:
                    while(colorSensor.getRed() < 150){
                        colorWheel.set(ControlMode.PercentOutput, power);
                    }
                    colorWheel.set(ControlMode.PercentOutput, 0);
                    break;
                case YELLOW:
                    while(colorSensor.getBlue() < 150){
                        colorWheel.set(ControlMode.PercentOutput, power);
                    }
                    colorWheel.set(ControlMode.PercentOutput, 0);
                    break;
            }
        }
    }

  public Colors getFieldTargetColor(){
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
        switch (gameData.charAt(0))
        {
        case 'B' :
            SmartDashboard.putString("ColorWheelTarget:" , "Blue");
            return Colors.BLUE;
        case 'G' :
            SmartDashboard.putString("ColorWheelTarget:" , "Green");
            return Colors.GREEN;
        case 'R' :
            SmartDashboard.putString("ColorWheelTarget:" , "Red");
            return Colors.RED;
        case 'Y' :
            SmartDashboard.putString("ColorWheelTarget:" , "Yellow");
            return Colors.YELLOW;
        }
    }
    return null;
  }

  
}
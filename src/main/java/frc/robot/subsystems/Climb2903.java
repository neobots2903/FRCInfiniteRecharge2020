//Climber - Top of robot, on both sides. Starts within robot frame, can extend up and then further outwards, clips to bar, and pulls down to lift robot.
//Hardware involved:
//- One pneumatic solenoid: for raising and lowering climb arms (MUST NOT lower arms if robot is climbing)
//- One larger pneumatic solenoid: for reaching towards bar and pulling robot up. (MUST NOT extend if smaller solenoids are closed)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb2903 extends SubsystemBase {

    public Solenoid extendLeft = new Solenoid(RobotMap.extendLeft);
    public Solenoid extendRight = new Solenoid(RobotMap.extendRight);
    public Solenoid armsRise = new Solenoid(RobotMap.armsRise);

    public void openExtendLeft() {
        extendLeft.set(true);
    }

    public void openExtendRight() {
        extendRight.set(true);

    }

    public void openArmsRise() {
        armsRise.set(true);
    }

    public Climb2903() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
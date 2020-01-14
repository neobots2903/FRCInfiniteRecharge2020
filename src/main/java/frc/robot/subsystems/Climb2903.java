//Climber - Top of robot, on both sides. Starts within robot frame, can extend up and then further outwards, clips to bar, and pulls down to lift robot.
//Hardware involved:
//- One pneumatic solenoid: for raising and lowering climb arms (MUST NOT lower arms if robot is climbing)
//- One larger pneumatic solenoid: for reaching towards bar and pulling robot up. (MUST NOT extend if smaller solenoids are closed)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb2903 extends SubsystemBase {

    public Solenoid armRiser;
    public Solenoid leftArm;
    public Solenoid rightArm;
    

    public Climb2903() {
        armRiser = new Solenoid(RobotMap.armRiser);
        leftArm = new Solenoid(RobotMap.leftArm);
        rightArm = new Solenoid(RobotMap.rightArm);
    }

    public void RaiseArms() {
        armRiser.set(true);
    }

    public void LowerArms() {
        armRiser.set(false);
    }

    public void ExtendLeft() {
        leftArm.set(true);
    }

    public void RetractLeft() {
        leftArm.set(false);
    }

    public void ExtendRight() {
        rightArm.set(true);
    }

    public void RetractRight() {
        rightArm.set(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
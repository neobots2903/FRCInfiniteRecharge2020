/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    public static final int TBD = 0;
    //Servos

    //Motors
    public static final int shooterWheelL = TBD;
    public static final int shooterWheelR = TBD;
    //Pneumatics
    public static final int extendLeft = 0; //Do this one first
    public static final int extendRight = 0; //And this one second
    public static final int armsRise = 0;
}
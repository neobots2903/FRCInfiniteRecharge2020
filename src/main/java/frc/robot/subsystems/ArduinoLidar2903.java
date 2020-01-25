/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoLidar2903 extends SubsystemBase {

    public SerialPort arduino;
    int leftDistance = 0;
    int leftStatus = 0;
    int centerDistance = 0;
    int centerStatus = 0;
    int rightDistance = 0;
    int rightStatus = 0;
    boolean noLidar = false;

    public ArduinoLidar2903() {
        try {
            arduino = new SerialPort(9600, SerialPort.Port.kUSB);
        } catch (Exception ex) {
            noLidar = true;
        }
    }

    public void updateData() {
        if (noLidar) return;
        String dist = arduino.readString();
        String[] distLines = dist.split("\r\n");
        String[] actuallyImportant = (distLines.length >= 7)
            ? new String[] { distLines[distLines.length - 1], distLines[distLines.length - 2],
                distLines[distLines.length - 3], distLines[distLines.length - 4], distLines[distLines.length - 5],
                distLines[distLines.length - 6], distLines[distLines.length - 7], }
            : distLines;
    
        for (String line : actuallyImportant)
            try {
                if (!line.isBlank() && !line.isEmpty()) {
                if (line.startsWith("D1:") && line.contains("!"))
                    leftDistance = Integer.parseInt(line.substring(3, line.indexOf("!")));
                else if (line.startsWith("D2:") && line.contains("!"))
                    centerDistance = Integer.parseInt(line.substring(3, line.indexOf("!")));
                else if (line.startsWith("D3:") && line.contains("!"))
                    rightDistance = Integer.parseInt(line.substring(3, line.indexOf("!")));
                else if (line.startsWith("S1:") && line.contains("!"))
                    leftStatus = Integer.parseInt(line.substring(3, line.indexOf("!")));
                else if (line.startsWith("S2:") && line.contains("!"))
                    centerStatus = Integer.parseInt(line.substring(3, line.indexOf("!")));
                else if (line.startsWith("S3:") && line.contains("!"))
                        rightStatus = Integer.parseInt(line.substring(3, line.indexOf("!")));
                    }
            } catch (Exception ex) {
        }
    }
    
    public enum LidarPosition {
        Left, Center, Right
    }
    
    public int getDistance(LidarPosition pos) {
        if (noLidar) return -1;
        updateData();
        if (pos == LidarPosition.Left)
            return leftDistance;
        else if (pos == LidarPosition.Center)
             return centerDistance;
        else if (pos == LidarPosition.Right)
            return rightDistance;
        else
            return 0;
    }
    
    public int getStatus(LidarPosition pos) { 
        if (noLidar) return 9;                  //if 0, distance is reliable
        updateData();                           //if 2, distance may be slightly off
        if (pos == LidarPosition.Left)          //if anything else, distance is unreliable
            return leftStatus;
        else if (pos == LidarPosition.Center)
            return centerStatus;
        else if (pos == LidarPosition.Right)
            return rightStatus;
        else
            return 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    
}

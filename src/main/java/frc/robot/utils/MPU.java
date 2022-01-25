package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MPUConstants;

public class MPU {
    private final MPUConstants constants = new MPUConstants();
    private final I2C MPU = new I2C(I2C.Port.kOnboard, constants.address);
    private Timer timer = new Timer();

    private double yaw = 0;
    private double previousTime = 0;

    public MPU(){
        MPU.write(0x6B, 0);
        timer.start();
    }

    public double getHeading(){
        byte[] byteArray = new byte[8];
        MPU.read(0x47, 8, byteArray);
        double raw = byteArray[0];
        raw += 1;
        yaw += 2 * raw * (timer.get() - previousTime);
        if (yaw > 180){
            yaw -= 360;
        } else if (yaw < -180){
            yaw += 360;
        }
        previousTime = timer.get();
        return yaw;
    }

    public Rotation2d getHeadingAsRotation2d(){
        byte[] byteArray = new byte[8];
        MPU.read(0x47, 8, byteArray);
        double raw = byteArray[0];
        raw += 1;
        yaw += 2 * raw * (timer.get() - previousTime);
        if (yaw > 180){
            yaw -= 360;
        } else if (yaw < -180){
            yaw += 360;
        }
        previousTime = timer.get();
        return Rotation2d.fromDegrees(yaw);
    }
}

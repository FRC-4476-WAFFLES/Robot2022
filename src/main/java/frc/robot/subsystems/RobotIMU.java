// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotIMU extends SubsystemBase {
  private final I2C MPU6050Gyro = new I2C(Port.kOnboard, Constants.MPUAddress);
  private final ADIS16448_IMU ADIS16448Gyro = new ADIS16448_IMU();
  private Timer timer = new Timer();
  private double previousTime = 0;
  private double currentTime = 0;
  private double elapsedTime = 0;

  private double[] accelerations = new double[3];
  private double[] rotations = new double[3];
  private double[] rotationRates = new double[3];

  private final GyroType gyroType;

  public enum GyroType{
    MPU6050,
    ADIS16448
  }

  /** Creates a new Gyro. */
  public RobotIMU(GyroType gyroType) {
    this.gyroType = gyroType;
    switch(gyroType) {
      case MPU6050:
        MPU6050Gyro.write(0x6B, 0);
        timer.start();
        break;
      case ADIS16448:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(gyroType){
      case MPU6050:
        updateMPU6050();
        break;
      case ADIS16448:
        updateADIS16448();
        break;
    }
  }

  public double[] getAccelerations() {
    return accelerations;
  }

  public double[] getRotations() {
    return rotations;
  }

  public double[] getRotationRates() {
    return rotationRates;
  }

  private void updateMPU6050(){
    previousTime = currentTime;
    currentTime = timer.get();
    elapsedTime = currentTime - previousTime;

    byte[] byteArray = new byte[14];
    MPU6050Gyro.read(0x3B, 14, byteArray);

    // Store the values of acceleration along the three axes in an array
    for (int x = 0; x < 3; x++) {
      accelerations[x] = (byteArray[x*2] << 8 | byteArray[x*2+1]) / 16384.0; //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    }

    // Calculating Roll and Pitch from the accelerometer data
    //accAngleX = (Math.atan(accY / Math.sqrt(Math.pow(accX, 2) + Math.pow(accZ, 2))) * 180 / Math.PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    //accAngleY = (Math.atan(-1 * accX / Math.sqrt(Math.pow(accY, 2) + Math.pow(accZ, 2))) * 180 / Math.PI) + 1.58; // AccErrorY ~(-1.58)

    // Store the values of rotation rates along the three axes in an array
    for (int x = 0; x < 3; x++) {
      rotationRates[x] = (byteArray[x*2] << 8 | byteArray[x*2+1]) / 16384.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    }

    for (int x = 0; x < 3; x++) {
      rotations[x] = rotations[x] + rotationRates[x] * elapsedTime;
    }
  }

  private void updateADIS16448() {
    accelerations[0] = ADIS16448Gyro.getAccelX();
    accelerations[1] = ADIS16448Gyro.getAccelY();
    accelerations[2] = ADIS16448Gyro.getAccelZ();
    rotations[0] = ADIS16448Gyro.getGyroAngleX();
    rotations[1] = ADIS16448Gyro.getGyroAngleY();
    rotations[2] = ADIS16448Gyro.getGyroAngleZ();
    rotationRates[0] = ADIS16448Gyro.getGyroRateX();
    rotationRates[1] = ADIS16448Gyro.getGyroRateY();
    rotationRates[2] = ADIS16448Gyro.getGyroRateZ();
  }
}

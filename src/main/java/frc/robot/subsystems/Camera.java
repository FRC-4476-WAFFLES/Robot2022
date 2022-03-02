/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  public enum CameraLEDMode {
    Default, Off, Strobe, On
  }

  public enum ProcessingMode {
    Vision, Driver
  }

  public enum SnapshotMode {
    SnapOff, SnapOn
  }

  public enum Pipeline {
    TheOnlyPipelineWeAreUsing
  }

  NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");

  //LinearFilter linearFilter = new LinearFilter(1, 2);

  private final LinearFilter taFilter = LinearFilter.movingAverage(25);
  private final LinearFilter txFilter = LinearFilter.movingAverage(20);

  private double taFiltered = 0;
  private double txFiltered = 0;

  /**
   * Creates a new CameraSubsystem.
   */
  public Camera() {

  }

  @Override
  public void periodic() {
    super.periodic();
    taFiltered = taFilter.calculate(getArea());
    txFiltered = txFilter.calculate(getHorizontal());
    // camera.getEntry("ledMode").setNumber(ledMode.ordinal());
    // camera.getEntry("camMode").setNumber(1);
    SmartDashboard.putBoolean("Camera has target", getHasTarget());
    SmartDashboard.putNumber("Camera tx", getHorizontal());
  }

  public void setLEDMode(CameraLEDMode mode) {
    camera.getEntry("ledMode").setNumber(mode.ordinal());
    System.out.println("Camera mode being set to: " + mode.ordinal());
  }

  public void setLEDOn() {
    camera.getEntry("ledMode").setNumber(4);
  }

  public void setLEDOff() {
    camera.getEntry("ledMode").setNumber(2);
  }

  public void setProcesingMode(ProcessingMode mode) {
    camera.getEntry("camMode").setNumber(mode.ordinal());
  }

  public void setPipeline(Pipeline line) {
    camera.getEntry("pipeline").setNumber(line.ordinal());
  }

  public boolean getHasTarget() {
    // tv: Whether the limelight has any valid targets (0 or 1)
    return camera.getEntry("tv").getDouble(0) != 0;
  }

  public double getHorizontal() {
    // tx: Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
    // degrees | LL2: -29.8 to 29.8 degrees)
    return camera.getEntry("tx").getDouble(0);
  }

  public double getFilteredHorizontal() {
    return txFiltered;
  }

  public double getVertical() {
    // ty: Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
    // degrees | LL2: -24.85 to 24.85 degrees)
    return camera.getEntry("ty").getDouble(0);
  }

  public double getArea() {
    // ta: Target Area (0% of image to 100% of image)
    return camera.getEntry("ta").getDouble(0);
  }

  public double getFilteredArea() {
    return taFiltered;
  }

  public double getSkew() {
    // ts: Skew or rotation (-90 degrees to 0 degrees)
    return camera.getEntry("ts").getDouble(0);
  }

  public double getLatency() {
    // tl: The pipeline’s latency contribution (ms) Add at least 11ms for image
    // capture latency.
    return camera.getEntry("tl").getDouble(0);
  }

  public double getShortSide() {
    // tshort: Sidelength of shortest side of the fitted bounding box (pixels)
    return camera.getEntry("tshort").getDouble(0);
  }

  public double getLongSide() {
    // tlong: Sidelength of shortest side of the fitted bounding box (pixels)
    return camera.getEntry("tlong").getDouble(0);
  }

  public double getHorizontalSide() {
    // thor: Horizontal sidelength of the rough bounding box (0 - 320 pixels)
    return camera.getEntry("thor").getDouble(0);
  }

  public double getVerticalSide() {
    // tvert: Vertical sidelength of the rough bounding box (0 - 320 pixels)
    return camera.getEntry("tvert").getDouble(0);
  }

  
  public Pipeline getActivePipeline() {
    // getpipe: True active pipeline index of the camera (0 .. 9)
    switch ((int) camera.getEntry("getpipe").getDouble(0)) {
    case 0:
      return Pipeline.TheOnlyPipelineWeAreUsing;
    default:
      return Pipeline.TheOnlyPipelineWeAreUsing;
    }
  }
}
/*
 * camtran: Results of a 3D position solution, 6 numbers: Translation (x,y,y)
 * Rotation(pitch,yaw,roll)
 */
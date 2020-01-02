/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Waypoint {

    public double x;
    public double y;
    public double theta;
    public MotionType motionType;
    public enum MotionType {
        LINEAR, ROTATION
    }

    public Waypoint(double x, double y) {
        this.x = x * Constants.DRIVE_REV_PER_INCH;
        this.y = y * Constants.DRIVE_REV_PER_INCH;
        this.motionType = MotionType.LINEAR;
    }

    public Waypoint(double theta) {
        this.theta = theta;
        this.motionType = MotionType.ROTATION;
    }
    

}

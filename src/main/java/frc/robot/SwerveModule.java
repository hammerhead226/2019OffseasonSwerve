/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;

public class SwerveModule {

    private CANSparkMax drive;
    private CANSparkMax steer;
    private EncoderWrapper steercoder;
    private int module;

    public SwerveModule(CANSparkMax drive, CANSparkMax steer, AnalogInput steercoder, int module) {
        this.drive = drive;
        this.steer = steer;
        this.steercoder = new EncoderWrapper(steercoder);
        this.module = module;
    }

    public void drive(double r, double theta) {
        steercoder.update();

        double error = theta - steercoder.getValue();
        error = error % Constants.ENCODER_TICKS;

        //choose direction to rotate module
        if(Math.abs(error) > Math.abs(error - Constants.ENCODER_TICKS)) {
            error = error - Constants.ENCODER_TICKS;
        } else if(Math.abs(error) > Math.abs(error + Constants.ENCODER_TICKS)) {
            error = error + Constants.ENCODER_TICKS;
        }
        //flip module or run in reverse
        if(Math.abs(error) > Math.abs(error - (Constants.ENCODER_TICKS / 2))) {
            r = -r;
            error = error - (Constants.ENCODER_TICKS / 2);
        } else if(Math.abs(error) > Math.abs(error + (Constants.ENCODER_TICKS / 2))) {
            r = -r;
            error = error + (Constants.ENCODER_TICKS / 2);
        }
        drive.set(r);
        steer.set(error * Constants.STEER_KP);
    }

    public boolean toPosition(double pos, double theta) {
        steercoder.update();

        double steerError = theta - steercoder.getValue();
        steerError = steerError % Constants.ENCODER_TICKS;

        //choose direction to rotate module
        if(Math.abs(steerError) > Math.abs(steerError - Constants.ENCODER_TICKS)) {
            steerError = steerError - Constants.ENCODER_TICKS;
        } else if(Math.abs(steerError) > Math.abs(steerError + Constants.ENCODER_TICKS)) {
            steerError = steerError + Constants.ENCODER_TICKS;
        }
        
        steer.set(steerError * Constants.STEER_KP);
        SwerveControl.errorTracker[module] = Math.abs(steerError);

        double driveError = pos - drive.getEncoder().getPosition();
        if(SwerveControl.trackerError()) {
            //limit max speed
            drive.set((Math.abs( driveError * Constants.DRIVE_KP) > Constants.MAX_AUTO_DRIVE_SPEED) ? Math.copySign(Constants.MAX_AUTO_DRIVE_SPEED , driveError * Constants.DRIVE_KP) : driveError * Constants.DRIVE_KP);
        } else {
            drive.set(0);
        }
        
        if(Math.abs(driveError) < Constants.MAX_AUTO_DRIVE_ERROR) {
            return true;
        } else return false;
    }

    public void resetDrive() {
        drive.getEncoder().setPosition(0);
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * Add your docs here.
 */
public class SwerveControl {

    private double r1, r2, r3, r4;
    private double theta1, theta2, theta3, theta4;
    private boolean isDone1, isDone2, isDone3, isDone4;
    private SwerveModule module1, module2, module3, module4;
    private PigeonIMU pigeon;
    private double lastX, lastY, lastTheta;
    private double holdAngle;
    private boolean isRotateZero = false;
    private double angleIntegrator;
    public static double[] errorTracker;
    private ArrayList<Boolean> isDones;

    public SwerveControl(SwerveModule module1, SwerveModule module2, SwerveModule module3, SwerveModule module4, PigeonIMU pigeon) {
        this.module1 = module1;
        this.module2 = module2;
        this.module3 = module3;
        this.module4 = module4;
        this.pigeon = pigeon;
        isDones = new ArrayList<>();
        errorTracker = new double[] {0, 0, 0, 0, 0};
    }

    public void control(double x, double y, double rotate) {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        double gyro = ypr[0];

        if(x == 0 && y == 0 && rotate == 0) {
            r1 = 0;
            r2 = 0;
            r3 = 0;
            r4 = 0;
        } else {
            if(!isRotateZero) {
                isRotateZero = (rotate == 0);
                if(isRotateZero) holdAngle = gyro;
            } else {
                isRotateZero = (rotate == 0);
            }

            if(rotate == 0) {
                double rotateError = holdAngle - gyro;
                rotate = -rotateError * Constants.DRIFT_CORRECTION_KP;
            }

            double mag = Math.hypot(x, y);
            double controllerTheta = Math.atan2(y, x);
            controllerTheta = (controllerTheta + 2 * Math.PI) % (2 * Math.PI);
            controllerTheta = controllerTheta - Math.toRadians(gyro);
            x = mag * Math.cos(controllerTheta);
            y = mag * Math.sin(controllerTheta);

            double a = rotate * Constants.DRIVE_BASE_LENGTH / 2 + x;
            double b = -rotate * Constants.DRIVE_BASE_LENGTH / 2 + x;
            double c = rotate * Constants.DRIVE_BASE_WIDTH / 2 + y;
            double d = -rotate * Constants.DRIVE_BASE_WIDTH / 2 + y;

            double[] normalizedMagnitude = Utility.normalizeMagnitude(
                Math.hypot(a, c), 
                Math.hypot(b, c), 
                Math.hypot(b, d), 
                Math.hypot(a, d)
            );

            r1 = normalizedMagnitude[0];
            r2 = normalizedMagnitude[1];
            r3 = normalizedMagnitude[2];
            r4 = normalizedMagnitude[3];

            theta1 = Utility.normalizeAngle(Math.atan2(c, a)) + Constants.MODULE_1_OFFSET;
            theta2 = Utility.normalizeAngle(Math.atan2(c, b)) + Constants.MODULE_2_OFFSET;
            theta3 = Utility.normalizeAngle(Math.atan2(d, b)) + Constants.MODULE_3_OFFSET;
            theta4 = Utility.normalizeAngle(Math.atan2(d, a)) + Constants.MODULE_4_OFFSET;
        }

        module1.drive(r1, theta1);
        module2.drive(r2, theta2);
        module3.drive(r3, theta3);
        module4.drive(r4, theta4);
    }

    public void toPosition(double x, double y) {
        double gyro = getGyro();

        if(x != lastX || y != lastY) {
            module1.resetDrive();
            module2.resetDrive();
            module3.resetDrive();
            module4.resetDrive();
            holdAngle = gyro;
        }

        lastX = x;
        lastY = y;

        double mag = Math.hypot(x, y);

        double controllerTheta = Math.atan2(y, x);
        controllerTheta = (controllerTheta + 2 * Math.PI) % (2 * Math.PI);
        controllerTheta = controllerTheta - Math.toRadians(gyro);
        x = mag * Math.cos(controllerTheta);
        y = mag * Math.sin(controllerTheta);

        double scaledX = x / mag;
        double scaledY = y / mag;

        double rotateError = gyro - holdAngle;
        double rotate = rotateError * Constants.DRIFT_CORRECTION_KP;

        double a = rotate * Constants.DRIVE_BASE_LENGTH / 2 + scaledX;
        double b = -rotate * Constants.DRIVE_BASE_LENGTH / 2 + scaledX;
        double c = rotate * Constants.DRIVE_BASE_WIDTH / 2 + scaledY;
        double d = -rotate * Constants.DRIVE_BASE_WIDTH / 2 + scaledY;

        theta1 = Utility.normalizeAngle(Math.atan2(c, a)) + Constants.MODULE_1_OFFSET;
        theta2 = Utility.normalizeAngle(Math.atan2(c, b)) + Constants.MODULE_2_OFFSET;
        theta3 = Utility.normalizeAngle(Math.atan2(d, b)) + Constants.MODULE_3_OFFSET;
        theta4 = Utility.normalizeAngle(Math.atan2(d, a)) + Constants.MODULE_4_OFFSET;

        isDone1 = module1.toPosition(Math.hypot(x, y), theta1);
        isDone2 = module2.toPosition(Math.hypot(x, y), theta2);
        isDone3 = module3.toPosition(Math.hypot(x, y), theta3);
        isDone4 = module4.toPosition(Math.hypot(x, y), theta4);
    }

    public void toAngle(double angle) {
        double gyro = getGyro();

        if(angle != lastTheta) {
            angleIntegrator = 0;
        }
        lastTheta = angle;
        
        double error = gyro - angle;
        if(Math.abs(error) < 5) {
            angleIntegrator += error;
        }

        isDone1 = (Math.abs(error) < Constants.MAX_AUTO_ROTATE_ERROR);
        isDone2 = (Math.abs(error) < Constants.MAX_AUTO_ROTATE_ERROR);
        isDone3 = (Math.abs(error) < Constants.MAX_AUTO_ROTATE_ERROR);
        isDone4 = (Math.abs(error) < Constants.MAX_AUTO_ROTATE_ERROR);
        control(0, 0, error * Constants.AUTO_ROTATE_KP + angleIntegrator * 0.001);
    }

    public boolean isDone() {
        boolean result = isDone1 && isDone2 && isDone3 && isDone4;
        if(result) {
            isDone1 = false;
            isDone2 = false;
            isDone3 = false;
            isDone4 = false;
        }

        isDones.add(result);
        
        boolean isTrulyDone = true;

        if(isDones.size() > 10) {
            for(int i = isDones.size() - 1; i > isDones.size() - 10; i--) {
                isTrulyDone &= isDones.get(i);
            }
        } else {
            return false;
        }

        if(isTrulyDone) {
            for(int i = 0; i < 10; i++) {
                isDones.add(false);
            }
        } 

        return isTrulyDone;
    }

    public static boolean trackerError() {
        double error = 0;
        boolean isReady = true;
        for(double d : errorTracker) {
            error += d;
            isReady = isReady && (d < Constants.MAX_AUTO_STEER_ERROR);
        }
        isReady = isReady && (error < 3 * Constants.MAX_AUTO_STEER_ERROR);
        return isReady;
    }

    private double getGyro() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

}

package frc.robot;

public class Utility {

    public static double normalizeAngle(double theta) {
        return -Constants.ENCODER_TICKS * ((theta + 2 * Math.PI) % (2 * Math.PI)) / (2 * Math.PI) + Constants.ENCODER_TICKS; 
    }

    public static double[] normalizeMagnitude(double r1, double r2, double r3, double r4) {
        double max = 1;
        if(r1 > max) max = r1;
        if(r2 > max) max = r2;
        if(r3 > max) max = r3;
        if(r4 > max) max = r4;

        return new double[] {r1 / max, r2 / max, r3 / max, r4 / max};
    }

}
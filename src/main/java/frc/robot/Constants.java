package frc.robot;

public class Constants {
    public static final int ENCODER_TICKS = 4096;
    public static final double DRIVE_REV_PER_INCH = .6614404704;

    public static final double DRIVE_BASE_WIDTH = 23.5 / 12;
    public static final double DRIVE_BASE_LENGTH = 21.5 / 12;

    public static final double STEER_KP = 0.0005;
    public static final double DRIVE_KP = 0.05;
    public static final double DRIFT_CORRECTION_KP = 0.015;
    public static final double AUTO_STEER_KP = 0.005;
    public static final double AUTO_ROTATE_KP = 0.005;

    public static final double MAX_AUTO_DRIVE_SPEED = 0.3;

    public static final double MAX_AUTO_DRIVE_ERROR = 1;
    public static final double MAX_AUTO_ROTATE_ERROR = 1;
    public static final double MAX_AUTO_STEER_ERROR = 400;

    public static final int MODULE_1_OFFSET = 1330;
    public static final int MODULE_2_OFFSET = 3656;
    public static final int MODULE_3_OFFSET = 620;
    public static final int MODULE_4_OFFSET = 3340;
}
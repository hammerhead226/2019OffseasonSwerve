package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class EncoderWrapper {

    private AnalogInput steercoder;
    private int lastEncoderValue;
    private int overflowCounter;

    public EncoderWrapper(AnalogInput steercoder) {
        this.steercoder = steercoder;
        lastEncoderValue = steercoder.getValue();
    }

    public void update() {
        int change = steercoder.getValue() - lastEncoderValue;

        //tracks large jumps == encoder overflow
        if(change > 2000) {
            overflowCounter--;
        }

        if(change < -2000) {
            overflowCounter++;
        }

        lastEncoderValue = steercoder.getValue();
    }

    public int getValue() {
        return Constants.ENCODER_TICKS * overflowCounter + steercoder.getValue();
    }

}
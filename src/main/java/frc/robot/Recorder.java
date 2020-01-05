/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class Recorder {

    private CANSparkMax[] drive;
    private AnalogInput[] encoders;
    private ArrayList<ArrayList<ModuleVector>> points;

    public Recorder(CANSparkMax[] drive, AnalogInput[] encoders) {
        assert(drive.length == encoders.length);
        this.drive = drive;
        this.encoders = encoders;    
        points = new ArrayList<ArrayList<ModuleVector>>();
        for(int i = 0; i < drive.length; i++) {
            points.add(new ArrayList<ModuleVector>());
        }
    }

    public void record() {
        for(int i = 0; i < drive.length; i++) {
            points.get(i).add(
                new ModuleVector(
                    drive[i].getEncoder().getPosition(), 
                    transformEncoder(
                        encoders[i].getValue(), Constants.MODULE_OFFSETS[i]
                    )
                )
            );
        }
    }

    public void writeToFile(String fileLocation) {

        File file = new File(fileLocation);

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(fileLocation));
            for(int i = 0; i < points.get(0).size(); i++) {
                writer.write(points.get(0).get(i).toString() + ";" + points.get(1).get(i).toString() + ";" + points.get(2).get(i).toString() + ";" + points.get(3).get(i).toString() + "\n");
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
		} finally {
            points = new ArrayList<ArrayList<ModuleVector>>();
            for(int i = 0; i < drive.length; i++) {
                points.add(new ArrayList<ModuleVector>());
            }
        }

    }

    public double transformEncoder(double position, double offset) {
        //System.out.println(position);
        System.out.println(((offset - position) / Constants.ENCODER_TICKS) * 360);
        return ((offset - position) / Constants.ENCODER_TICKS) * 360;
    }

    private class ModuleVector {

        private double position;
        private double angle;

        private ModuleVector(double position, double angle) {
            this.position = position;
            this.angle = angle;
        }

        public String toString() {
            System.out.println(angle);
            return position + "," + angle;
        }

    }

}

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class CSVWriter {

    private final BufferedWriter writer;

    public CSVWriter() {
        String filePath = Environment.getExternalStorageDirectory().getPath() + "/output.csv";
        try {
            writer = new BufferedWriter(new FileWriter(filePath));
            writer.append("Runtime,Target Power,Flywheel Velocity,Battery Voltage");
            writer.newLine();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void write(String string) {
        try {
            writer.append(string);
            writer.newLine();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
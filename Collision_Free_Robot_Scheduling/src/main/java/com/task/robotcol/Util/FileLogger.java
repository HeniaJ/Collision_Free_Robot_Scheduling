package com.task.robotcol.Util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class FileLogger {
    private final String filename;

    public FileLogger(String filename) {
        this.filename = filename;
    }

    public void log(int pathLength, int robots, int tasks, int steps, int sumTaskLength) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename, true))) {
            writer.write(pathLength + " " + robots + " " + tasks + " " + steps + " " + sumTaskLength);
            writer.newLine();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

package com.task.robotcol.Model;

import javafx.event.Event;

public class SimulationEndedEventArgs extends Event {
    final int stepsAndTasksTime;

    public SimulationEndedEventArgs(int stepsAndTasksTime) {
        super(Event.ANY);
        this.stepsAndTasksTime = stepsAndTasksTime;
    }

    public int getStepsAndTasksTime() {
        return stepsAndTasksTime;
    }
}

package com.task.robotcol.Model;

import javafx.event.Event;

public class RobotEventArgs extends Event {
    private final int nextIndex;
    private final int prevIndex;
    private Integer curTaskIndex = null;
    private Integer curTaskLength = null;
    private final int robotNum;

    public RobotEventArgs(int nextIndex, int prevIndex, int curTaskIndex, int curTaskLength, int robotNum) {
        super(Event.ANY);
        this.nextIndex = nextIndex;
        this.prevIndex = prevIndex;
        this.robotNum = robotNum;
        this.curTaskIndex = Integer.valueOf(curTaskIndex);
        this.curTaskLength = Integer.valueOf(curTaskLength);
    }

    public RobotEventArgs(int nextIndex, int prevIndex, int robotNum) {
        super(Event.ANY);
        this.nextIndex = nextIndex;
        this.prevIndex = prevIndex;
        this.robotNum = robotNum;
    }

    public Integer getCurTaskIndex() {
        return curTaskIndex;
    }

    public Integer getCurTaskLength() {
        return curTaskLength;
    }

    public int getNextIndex() {
        return nextIndex;
    }

    public int getPrevIndex() {
        return prevIndex;
    }

    public int getRobotNum() {
        return robotNum;
    }
}
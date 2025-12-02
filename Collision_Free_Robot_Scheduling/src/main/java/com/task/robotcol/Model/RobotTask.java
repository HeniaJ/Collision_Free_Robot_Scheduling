package com.task.robotcol.Model;

public class RobotTask {
    private final int length;
    private int remainingLength;
    private final int index;
    private final int taskNum;

    public RobotTask(int index, int taskNum, int length) {
        this.length = length;
        this.remainingLength = length;
        this.index = index;
        this.taskNum = taskNum;
    }

    public RobotTask(int index, int taskNum) {
        this.taskNum = taskNum;
        this.length = 1;
        this.remainingLength = length;
        this.index = index;
    }

    public int getIndex() {
        return index;
    }

    public boolean isFinished() {
        return remainingLength<=0;
    }

    public int getRemainingLength() {
        return remainingLength;
    }

    public void decrementRemainingLength() {
        --remainingLength;
    }

    public int getTaskNum() {
        return taskNum;
    }

    public int getLength() {
        return length;
    }

}

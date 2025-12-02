package com.task.robotcol.Model;
import javafx.event.Event;
import javafx.event.EventHandler;

import java.util.ArrayList;

public class Robot {
    private int index;
    private final int startIndex;
    private boolean startFromFirst;
    private boolean isFinished = true;
    private int stepsAndTasksTime = 0;
    private final int robotNum;
    private ArrayList<RobotTask> tasks = new ArrayList<>();
    private final RobotTaskManager robotTaskManager = new RobotTaskManager();
    private boolean isRobotNotAtStartPosition = true;
    public EventHandler<RobotEventArgs> gameAdvanced;
    public Robot(int startIndex, int robotNum) {
        this.index = startIndex;
        this.startIndex = startIndex;
        this.robotNum = robotNum;
    }

    public int getIndex() {
        return  index;
    }

    public int getStartIndex() {
        return startIndex;
    }

    public void setTasks(ArrayList<RobotTask> tasks) {
        this.tasks = tasks;
        if(tasks!=null && !tasks.isEmpty()) {
            isFinished = false;
        }
    }

    public void setStartFromFirst(Boolean startFromFirst) {
        this.startFromFirst = startFromFirst;
        if((startFromFirst && tasks.getFirst().getIndex()>=index) ||
                (!startFromFirst && tasks.getLast().getIndex()<=index)) {
            isRobotNotAtStartPosition = false;
        }
    }

    public ArrayList<RobotTask> getTasks() {
        return tasks;
    }

    public boolean makeAMove() {
        if(isFinished || tasks.isEmpty()) return false;
        RobotTask currTask = null;
        int prevIndex = index;
        if(startFromFirst) {
            if(isRobotNotAtStartPosition) {
                //meg nem ert elore a robot
                index--;
                isRobotNotAtStartPosition = !tasks.getFirst().isFinished() && index!=tasks.getFirst().getIndex();
            } else {
                //elore ert a robot
                currTask = robotTaskManager.getTaskOnIndex(index, tasks);
                if(currTask!=null && currTask.getRemainingLength()>0) {
                    //nem fejezte be meg taskot amin all
                    currTask.decrementRemainingLength();
                } else {
                    index++;
                }
            }
        } else {
            if(isRobotNotAtStartPosition) {
                index++;
                isRobotNotAtStartPosition = !tasks.getLast().isFinished() && index != (tasks.getLast().getIndex());
            } else {
                currTask = robotTaskManager.getTaskOnIndex(index, tasks);
                if(currTask!=null && currTask.getRemainingLength()>0) {
                    //nem fejezte be meg taskot amin all
                    currTask.decrementRemainingLength();
                } else {
                    index--;
                }
            }
        }
        stepsAndTasksTime++;
        setFinished();
        handleGameStep(currTask, prevIndex);
        return true;
    }

    private void handleGameStep(RobotTask currTask, int prevIndex) {
        if(currTask==null) {
            gameAdvanced.handle(new RobotEventArgs(index, prevIndex, robotNum));
        } else {
            gameAdvanced.handle(new RobotEventArgs(
                    index,
                    prevIndex,
                    Integer.valueOf(currTask.getIndex()),
                    Integer.valueOf(currTask.getRemainingLength()),
                    robotNum
            ));
        }
    }

    private void setFinished() {
        isFinished = tasks.stream().allMatch(RobotTask::isFinished);
    }

    public boolean isFinished() {
        return isFinished;
    }

    public int getStepsAndTasksTime() {
        return stepsAndTasksTime;
    }

    public int getRobotNum() {
        return robotNum;
    }
}

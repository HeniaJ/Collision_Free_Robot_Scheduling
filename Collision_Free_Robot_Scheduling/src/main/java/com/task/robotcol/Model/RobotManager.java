package com.task.robotcol.Model;

import java.util.List;

public class RobotManager {
    public Robot getRobotOnIndex(int index, List<Robot> robots) {
        return robots.stream()
                .filter(robot -> robot.getIndex()==index)
                .findFirst()
                .orElse(null);
    }
}

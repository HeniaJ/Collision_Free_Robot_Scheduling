package com.task.robotcol.Controller;

import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Circle;
import javafx.scene.text.Text;

import static javafx.scene.paint.Color.*;

public class PathNode extends StackPane {
    private Integer robotNum;
    private final Integer taskNum;
    private Integer taskLength;
    private final int vertexNum;
    private final Circle circle;

    public PathNode(Integer robotNum, Integer taskNum, Integer taskLength, int vertexNum, int columns, double gaps, double circleRadius) {
        circle = new Circle(circleRadius, Color.web(taskNum==null ? "#D7BCE8" : "#E8CEE4"));
        this.robotNum = robotNum;
        this.taskNum = taskNum;
        this.taskLength = taskLength;
        this.vertexNum = vertexNum;
        int x = (vertexNum % columns);
        int y = (vertexNum / columns);
        if(y % 2 == 1) {
            x = columns - x - 1 ;
        }
        this.setLayoutX(x*gaps);
        this.setLayoutY(y*gaps);
        refreshText();
    }

    public void refreshText() {
        Text text = new Text();
        String strForNode = "V" + vertexNum;
        if (robotNum!=null) {
            strForNode += ", R" + robotNum;
            circle.setStroke(Color.web("#8884FF"));
            circle.setStrokeWidth(5);
        } else {
            circle.setStroke(Color.web("#5D576B"));
            circle.setStrokeWidth(5);
        }
        if(taskLength!=null) {
            strForNode += (", T" + taskNum + ": " + taskLength);
        }
        text.setText(strForNode);
        this.getChildren().setAll(circle, text);
    }

    public void setTaskLength(Integer taskLength) {
        this.taskLength = taskLength;
    }

    public void setRobotNum(Integer robotNum) {
        this.robotNum = robotNum;
    }
    public Integer getRobotNum() {
        return robotNum;
    }

    public Circle getCircle() {
        return circle;
    }
}

package com.task.robotcol.Controller;

import javafx.scene.paint.Color;
import javafx.scene.shape.Line;

public class Edge {
    private final Line line;
    private double calcXPos(int vertexNum, int columns, double gaps, double circleRadius) {
        int x = (vertexNum % columns);
        int y = (vertexNum / columns);
        if(y % 2 == 1) {
            x = columns - x - 1 ;
        }
        return x*gaps+circleRadius;
    }

    private double calcYPos(int vertexNum, int columns, double gaps, double circleRadius) {
        int y = (vertexNum / columns);
        return y*gaps+circleRadius;
    }
    public Edge(int vertexNum, int columns, double gaps, double circleRadius) {
        line = new Line(
                calcXPos(vertexNum,columns,gaps,circleRadius),
                calcYPos(vertexNum,columns,gaps,circleRadius),
                calcXPos(vertexNum+1,columns,gaps,circleRadius),
                calcYPos(vertexNum+1,columns,gaps,circleRadius));
        line.setStroke(Color.web("#5D576B"));
        line.setStrokeWidth(5);
    }

    public Line getLine() {
        return line;
    }
}

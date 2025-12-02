package com.task.robotcol.Controller;

import com.task.robotcol.Model.*;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollPane;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.text.Text;

import java.util.*;

import static javafx.scene.paint.Color.*;

public class RobotColController {
    KRobotPathGraph kRobotPathGraph;
    private final List<PathNode> pathNodes = new ArrayList<>();
    private final int columns = 10;
    private final double circleRadius = 50.0;
    private final double gaps = 120.0;
    private int pathLength;
    private int robotCount;
    private int taskCount;
    private boolean isTaskLengthRandom;
    private Label simulationStatusLabel;
    public Button newSimulationButton = new Button("New Simulation");
    private Scene scene;

    private void setRobotsAndTasksInModel() {
        pathNodes.clear();
        simulationStatusLabel = new Label();
        kRobotPathGraph = new KRobotPathGraph(pathLength, placeTasks(), placeRobots());
        kRobotPathGraph.setStepHandlers(robotEventArgs -> {
            if (robotEventArgs.getPrevIndex() != robotEventArgs.getNextIndex()) {
                PathNode prevNode = pathNodes.get(robotEventArgs.getPrevIndex());
                PathNode nextNode = pathNodes.get(robotEventArgs.getNextIndex());
                nextNode.setRobotNum(Integer.valueOf(robotEventArgs.getRobotNum()));
                nextNode.refreshText();
                if(prevNode.getRobotNum()==robotEventArgs.getRobotNum()) {
                    prevNode.setRobotNum(null);
                    prevNode.refreshText();
                }
            }
            if (robotEventArgs.getCurTaskIndex() != null && robotEventArgs.getCurTaskLength() != null) {
                PathNode taskNode = pathNodes.get(robotEventArgs.getCurTaskIndex());
                taskNode.setTaskLength(robotEventArgs.getCurTaskLength());
                taskNode.refreshText();
            }
        });

        kRobotPathGraph.simulationEnded = simulationEndedEventArgs -> {
            simulationStatusLabel.setText("Simulation ended in " + simulationEndedEventArgs.getStepsAndTasksTime() + " steps.");
        };
    }

    private ArrayList<Integer> placeRobots() {
        Random random = new Random();
        Set<Integer> uniquePositions = new HashSet<>();

        while (uniquePositions.size() < robotCount) {
            uniquePositions.add(Integer.valueOf(random.nextInt(pathLength)));
        }

        return new ArrayList<Integer>(uniquePositions);
    }

    private Map<Integer, Integer> placeTasks() {
        Random random = new Random();
        Map<Integer, Integer> uniquePositions = new HashMap<Integer, Integer>();
        int taskLength;

        while (uniquePositions.size() < taskCount) {
            if (isTaskLengthRandom) {
                taskLength = random.nextInt(15) + 1;
            } else {
                taskLength = 1;
            }
            uniquePositions.put(Integer.valueOf(random.nextInt(pathLength)), Integer.valueOf(taskLength));
        }

        return uniquePositions;
    }

    public ScrollPane initializeGraph(int pathLength, int robotCount, int taskCount, boolean isTaskLengthRandom) {
        this.pathLength = pathLength;
        this.robotCount = robotCount;
        this.taskCount = taskCount;
        this.isTaskLengthRandom = isTaskLengthRandom;
        setRobotsAndTasksInModel();
        return createScrollPane();
    }

    private Pane createPathNodesAndLines() {
        Pane pane = new Pane();
        for (int i = 0; i < kRobotPathGraph.getPathLength(); i++) {
            if(i!=kRobotPathGraph.getPathLength()-1) {
                pane.getChildren().add((new Edge(i, columns, gaps, circleRadius)).getLine());
            }
            pane.getChildren().add(createPathNodeOnIndex(i));
        }
        return pane;
    }

    private PathNode createPathNodeOnIndex(int i) {
        PathNode pathNode = new PathNode(
                kRobotPathGraph.getRobotNum(i),
                kRobotPathGraph.getTaskNum(i),
                kRobotPathGraph.getTaskLength(i),
                i,
                columns,
                gaps,
                circleRadius);
        pathNodes.add(pathNode);
        return pathNode;
    }
    private ScrollPane createScrollPane() {
        ScrollPane scrollPane = new ScrollPane(createVBox());
        scrollPane.setFitToWidth(true);
        scrollPane.setVbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);
        return scrollPane;
    }

    private VBox createVBox() {
        VBox vbox = new VBox(10, createPathNodesAndLines(), createForwardButton(), newSimulationButton, simulationStatusLabel);
        vbox.setPadding(new Insets(15,30,15,30));
        BackgroundFill backgroundFill = new BackgroundFill(WHITE, CornerRadii.EMPTY, Insets.EMPTY);
        vbox.setBackground(new Background(backgroundFill));
        return vbox;
    }

    private Button createForwardButton() {
        Button forwardButton = new Button("Step forward");
        forwardButton.setOnAction(event -> onHelloButtonClick());
        return forwardButton;
    }

    @FXML
    private void onHelloButtonClick() {
        kRobotPathGraph.makeAMove();
    }
}
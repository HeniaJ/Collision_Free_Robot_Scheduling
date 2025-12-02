package com.task.robotcol.Controller;

import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

public class RobotStage {
    private final RobotColController robotColController = new RobotColController();
    private final Stage stage;
    private final StackPane stackPane;

    public RobotStage(Stage stage) {
        this.stage = stage;
        stage.setTitle("Robots and Tasks");
        stage.setMaximized(true);
        stackPane = new StackPane();
        createInputForm();
        stage.setScene(new Scene(stackPane));
        robotColController.newSimulationButton.setOnAction(actionEvent -> {
                createInputForm();
        });
    }

    private void createInputForm() {
        VBox formLayout = new VBox(10);
        formLayout.setAlignment(Pos.CENTER);

        HBox inputLayout = new HBox(10);
        inputLayout.setAlignment(Pos.CENTER);

        Label verticesLabel = new Label("Vertices:");
        TextField verticesInput = new TextField();
        verticesInput.setPrefWidth(50);

        Label tasksLabel = new Label("Tasks:");
        TextField tasksInput = new TextField();
        tasksInput.setPrefWidth(50);

        Label robotsLabel = new Label("Robots:");
        TextField robotsInput = new TextField();
        robotsInput.setPrefWidth(50);

        CheckBox customParamCheckBox = new CheckBox("Different Task Length");

        Label errorLabel = new Label();
        errorLabel.setStyle("-fx-text-fill: red;");

        Button generateButton = createGenerateButton(verticesInput, tasksInput, robotsInput, customParamCheckBox, errorLabel);

        inputLayout.getChildren().addAll(
                verticesLabel, verticesInput,
                tasksLabel, tasksInput,
                robotsLabel, robotsInput,
                customParamCheckBox, generateButton
        );
        formLayout.getChildren().addAll(inputLayout, errorLabel);
        stackPane.getChildren().setAll(formLayout);
    }

    private Button createGenerateButton(TextField verticesInput, TextField tasksInput, TextField robotsInput, CheckBox customParamCheckBox, Label errorLabel) {
        Button generateButton = new Button("Generate");
        generateButton.setOnAction(event -> {
            try {
                int vertices = Integer.parseInt(verticesInput.getText());
                int tasks = Integer.parseInt(tasksInput.getText());
                int robots = Integer.parseInt(robotsInput.getText());
                boolean customParam = customParamCheckBox.isSelected();

                if(robots < 1 || tasks < 1) {
                    errorLabel.setText("Error: The number of robots and tasks must be at least 1");
                }
                else if (vertices < robots || vertices < tasks) {
                    errorLabel.setText("Error: The number of vertices must be greater than or equal to the number of robots and tha number of tasks.");
                } else {
                    errorLabel.setText("");
                    stackPane.getChildren().setAll(robotColController.initializeGraph(vertices, robots, tasks, customParam));
                }
            } catch (NumberFormatException e) {
                errorLabel.setText("Invalid input, please enter integers.");
            }
        });
        return generateButton;
    }


    public Stage getStage() {
        return stage;
    }
}

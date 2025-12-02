package com.task.robotcol;

import com.task.robotcol.Controller.RobotColController;
import com.task.robotcol.Controller.RobotStage;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

import java.io.IOException;

public class RobotColApplication extends Application {
    RobotStage robotStage;
    @Override
    public void start(Stage stage) throws Exception {
        robotStage = new RobotStage(stage);
        stage.show();
    }

    public static void main(String[] args) {
        launch();
    }
}
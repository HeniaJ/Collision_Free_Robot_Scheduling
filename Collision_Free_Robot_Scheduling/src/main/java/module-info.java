module com.task.robotcol {
    requires javafx.controls;
    requires javafx.fxml;

    requires org.kordamp.bootstrapfx.core;
    requires java.logging;

    opens com.task.robotcol to javafx.fxml;
    exports com.task.robotcol;
    exports com.task.robotcol.Controller;
    opens com.task.robotcol.Controller to javafx.fxml;
}
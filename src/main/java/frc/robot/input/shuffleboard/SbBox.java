package frc.robot.input.shuffleboard;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;

// TODO finish implementation of user-defined grid layouts that assigns equal space to every component inside and can resize at will
public class SbBox {
    @FunctionalInterface
    private static interface ComponentUpdater {
        void update(int x, int y, int width, int height);
    }

    private final ShuffleboardContainer container;

    private final ComponentUpdater[][] updaters;
    private final int columns, rows;

    private int x, y, width, height;

    // init
    public SbBox(ShuffleboardContainer container, int x, int y, int width, int height, int columns, int rows) {
        this.container = container;

        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;

        this.columns = columns;
        this.rows = rows;

        this.updaters=  new ComponentUpdater[columns][rows];
    }

    public SbBox box(int localX, int localY, int boxWidth, int boxHeight, int columns, int rows) {
        return new SbBox(container, x + localX, x + localY, boxWidth, boxHeight, columns, rows);
    }

    public void update(int x, int y, int width, int height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;

        
    }

    // widgets
    public ComplexWidget complexWidget(WidgetType type, String title, Sendable sendable, int localX, int localY, int widgetWidth, int widgetHeight, Map<String, Object> properties) {
        return container
            .add(title, sendable)
            .withWidget(type)
            .withPosition(x + localX, y + localY)
            .withSize(widgetWidth, widgetHeight)
            .withProperties(properties);
    }

    public SimpleWidget simpleWidget(WidgetType type, String title, Object initial, int localX, int localY, int widgetWidth, int widgetHeight, Map<String, Object> properties) {
        return container
            .add(title, initial)
            .withWidget(type)
            .withPosition(x + localX, y + localY)
            .withSize(widgetWidth, widgetHeight)
            .withProperties(properties);
    }

    public SimpleWidget booleanBox(String title, boolean initial, String colorWhenTrue, String colorWhenFalse, int localX, int localY, int widgetWidth, int widgetHeight) {
        return simpleWidget(BuiltInWidgets.kBooleanBox, title, initial, localX, localY, widgetWidth, widgetHeight, Map.of("Color when true", colorWhenTrue, "Color when false", colorWhenFalse));
    }

    public SimpleWidget booleanBox(String title, boolean initial, int localX, int localY, int widgetWidth, int widgetHeight) {
        return simpleWidget(BuiltInWidgets.kBooleanBox, title, initial, localX, localY, widgetWidth, widgetHeight, Map.of());
    }

    public SimpleWidget slider(String title, double initial, double min, double max, int localX, int localY, int widgetWidth, int widgetHeight) {
        return simpleWidget(BuiltInWidgets.kNumberSlider, title, initial, localX, localY, widgetWidth, widgetHeight, Map.of("Min", min, "Max", max));
    }

    public SimpleWidget bar(String title, double initial, double center, double min, double max, int localX, int localY, int widgetWidth, int widgetHeight) {
        return simpleWidget(BuiltInWidgets.kNumberBar, title, initial, localX, localY, widgetWidth, widgetHeight, Map.of("Min", min, "Max", max, "Center", center));
    }

    public SimpleWidget dial(String title, int initial, int min, int max, boolean show, int localX, int localY, int widgetWidth, int widgetHeight) {
        return simpleWidget(BuiltInWidgets.kDial, title, initial, localX, localY, widgetWidth, widgetHeight, Map.of("Min", min, "Max", max, "Show value", show));
    }

    public ComplexWidget command(String title, Command command, int localX, int localY, int widgetWidth, int widgetHeight) {
        return complexWidget(BuiltInWidgets.kCommand, title, command, localX, localY, widgetWidth, widgetHeight, Map.of());
    }
}

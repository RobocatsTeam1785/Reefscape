package frc.lib.utility;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;

public class ShuffleboardUtils {
    public static ComplexWidget complexWidget(WidgetType type, ShuffleboardContainer container, String title, Sendable sendable, Map<String, Object> properties) {
        return container
            .add(title, sendable)
            .withWidget(type)
            .withProperties(properties);
    }

    public static ComplexWidget complexWidget(WidgetType type, ShuffleboardContainer container, String title, Sendable sendable) {
        return container
            .add(title, sendable)
            .withWidget(type);
    }

    public static SimpleWidget simpleWidget(WidgetType type, ShuffleboardContainer container, String title, Object initial, Map<String, Object> properties) {
        return container
            .add(title, initial)
            .withWidget(type)
            .withProperties(properties);
    }

    public static SimpleWidget simpleWidget(WidgetType type, ShuffleboardContainer container, String title, Object initial) {
        return container
            .add(title, initial)
            .withWidget(type);
    }

    public static SimpleWidget booleanBox(ShuffleboardContainer container, String title, boolean initial, String colorWhenTrue, String colorWhenFalse) {
        return simpleWidget(BuiltInWidgets.kBooleanBox, container, title, initial, Map.of("Color when true", colorWhenTrue, "Color when false", colorWhenFalse));
    }

    public static SimpleWidget booleanBox(ShuffleboardContainer container, String title, boolean initial) {
        return simpleWidget(BuiltInWidgets.kBooleanBox, container, title, initial);
    }

    public static SimpleWidget slider(ShuffleboardContainer container, String title, double initial, double min, double max) {
        return simpleWidget(BuiltInWidgets.kNumberSlider, container, title, initial, Map.of("Min", min, "Max", max));
    }

    public static SimpleWidget bar(ShuffleboardContainer container, String title, double initial, double center, double min, double max) {
        return simpleWidget(BuiltInWidgets.kNumberBar, container, title, initial, Map.of("Min", min, "Max", max, "Center", center));
    }

    public static SimpleWidget dial(ShuffleboardContainer container, String title, int initial, int min, int max, boolean show) {
        return simpleWidget(BuiltInWidgets.kDial, container, title, initial, Map.of("Min", min, "Max", max, "Show value", show));
    }

    public static ComplexWidget command(ShuffleboardContainer container, String title, Command command) {
        return complexWidget(BuiltInWidgets.kCommand, container, title, command);
    }
}

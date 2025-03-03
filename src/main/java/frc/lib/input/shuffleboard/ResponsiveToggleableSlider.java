package frc.lib.input.shuffleboard;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.utility.ShuffleboardUtils;

public class ResponsiveToggleableSlider {
    // component classes
    public static interface Feedback {
        public void update();
    }

    public static class SingleFeedback implements Feedback {
        private final DoubleSupplier responseSupplier;
        private final GenericEntry responseBarEntry;

        public SingleFeedback(
            ShuffleboardContainer container,
            int columnIndex, int rowIndex,
            DoubleSupplier responseSupplier,
            double initialValue, double centerValue, double minValue, double maxValue
        ) {
            this.responseBarEntry = ShuffleboardUtils.bar(container, "Response", initialValue, centerValue, minValue, maxValue)
                .withPosition(columnIndex, rowIndex)
                .getEntry();
            
            this.responseSupplier = responseSupplier;
        }

        @Override
        public void update() {
            double response = responseSupplier.getAsDouble();
            responseBarEntry.setDouble(response);
        }
    }

    public static class DualFeedback implements Feedback {
        private final ShuffleboardLayout grid;
        private final DoubleSupplier leftResponseSupplier, rightResponseSupplier;

        private final GenericEntry leftResponseBarEntry, rightResponseBarEntry;

        public DualFeedback(
            ShuffleboardContainer container,
            int columnIndex, int rowIndex,
            DoubleSupplier leftResponseSupplier, DoubleSupplier rightResponseSupplier,
            double initialValue, double centerValue, double minValue, double maxValue
        ) {
            this.grid = container.getLayout("Dual Feedback", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"))
                .withPosition(columnIndex, rowIndex);

            this.leftResponseBarEntry = ShuffleboardUtils.bar(grid, "Left Response", initialValue, centerValue, minValue, maxValue)
                .withPosition(0, 0)
                .getEntry();
            
            this.rightResponseBarEntry = ShuffleboardUtils.bar(grid, "Right Response", initialValue, centerValue, minValue, maxValue)
                .withPosition(1, 0)
                .getEntry();
            
            this.leftResponseSupplier = leftResponseSupplier;
            this.rightResponseSupplier = rightResponseSupplier;
        }

        @Override
        public void update() {
            double leftResponse = leftResponseSupplier.getAsDouble();
            leftResponseBarEntry.setDouble(leftResponse);

            double rightResponse = rightResponseSupplier.getAsDouble();
            rightResponseBarEntry.setDouble(rightResponse);
        }
    }

    // constants
    public static final int WIDTH = 15;
    public static final int HEIGHT = 12;

    // control
    private final GenericEntry controlSliderEntry;
    @SuppressWarnings("unused")
    private final ComplexWidget toggle;

    // feedback
    private final Feedback feedback;
    private final GenericEntry enabledBoxEntry;

    // values
    private final DoubleConsumer valueConsumer;

    private final double safeValue;

    // state
    private boolean enabled;

    // initialization
    /** initializes a 15x12 toggleable slider with feedback in a grid layout at the specified position */
    private ResponsiveToggleableSlider(
        ShuffleboardContainer grid, Feedback feedback,
        DoubleConsumer valueConsumer,

        double safeValue, double initialValue, double centerValue, double minValue, double maxValue,
        boolean initiallyEnabled
    ) {
        // feedback
        this.feedback = feedback;
        
        this.enabledBoxEntry = ShuffleboardUtils.booleanBox(grid, "Enabled", initiallyEnabled)
            .withPosition(1, 1)
            .getEntry();

        // control
        Command toggleCommand = new InstantCommand(this::toggle);
        toggleCommand.setName("Toggle");

        this.controlSliderEntry = ShuffleboardUtils.slider(grid, "Slider", initialValue, minValue, maxValue)
            .withPosition(0, 0)
            .getEntry();
        
        this.toggle = ShuffleboardUtils.command(grid, "Toggle", toggleCommand)
            .withPosition(1, 0);
        
        // values
        this.valueConsumer = valueConsumer;
        this.safeValue = safeValue;
    }

    // commands
    private void toggle() {
        enabled = !enabled;
        enabledBoxEntry.setBoolean(enabled);
    }

    // periodic
    public void periodic() {
        if (enabled) {
            double control = controlSliderEntry.getDouble(safeValue);
            valueConsumer.accept(control);
        }

        feedback.update();
    }

    // static initializers
    public static ResponsiveToggleableSlider singleFeedbackSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        DoubleConsumer valueConsumer, DoubleSupplier responseSupplier,

        double safeValue, double initialValue, double centerValue, double minValue, double maxValue,
        boolean initiallyEnabled
    ) {
        ShuffleboardLayout layout = container
            .getLayout(title, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "HIDDEN"))
            .withSize(WIDTH, HEIGHT)
            .withPosition(columnIndex, rowIndex);

        SingleFeedback feedback = new SingleFeedback(layout, 0, 1, responseSupplier, initialValue, centerValue, minValue, maxValue);

        return new ResponsiveToggleableSlider(layout, feedback, valueConsumer, safeValue, initialValue, centerValue, minValue, maxValue, initiallyEnabled);
    }

    public static ResponsiveToggleableSlider dualFeedbackSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        DoubleConsumer valueConsumer, DoubleSupplier leftResponseSupplier, DoubleSupplier rightResponseSupplier,

        double safeValue, double initialValue, double centerValue, double minValue, double maxValue,
        boolean initiallyEnabled
    ) {
        ShuffleboardLayout layout = container
            .getLayout(title, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "HIDDEN"))
            .withSize(WIDTH, HEIGHT)
            .withPosition(columnIndex, rowIndex);

        DualFeedback feedback = new DualFeedback(layout, 0, 1, leftResponseSupplier, rightResponseSupplier, initialValue, centerValue, minValue, maxValue);

        return new ResponsiveToggleableSlider(layout, feedback, valueConsumer, safeValue, initialValue, centerValue, minValue, maxValue, initiallyEnabled);
    }

    /** <b>only use U classes for which U.of returns M</b> */
    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> ResponsiveToggleableSlider unitSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        Consumer<M> valueConsumer, Supplier<M> responseSupplier, U unit,
        double safeValue, double initialValue, double centerValue, double minValue, double maxValue, boolean initiallyEnabled
    ) {
        return singleFeedbackSlider(
            container, title,
            columnIndex, rowIndex,
            velocity -> valueConsumer.accept((M)unit.of(velocity)), () -> responseSupplier.get().in(unit),
            safeValue, initialValue, centerValue, minValue, maxValue, initiallyEnabled
        );
    }

    /** <b>only use U classes for which U.of returns M</b> */
    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> ResponsiveToggleableSlider unitSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        Consumer<M> valueConsumer, Supplier<M> leftResponseSupplier, Supplier<M> rightResponseSupplier, U unit,
        double safeValue, double initialValue, double centerValue, double minValue, double maxValue, boolean initiallyEnabled
    ) {
        return dualFeedbackSlider(
            container, title,
            columnIndex, rowIndex,
            velocity -> valueConsumer.accept((M)unit.of(velocity)), () -> leftResponseSupplier.get().in(unit), () -> rightResponseSupplier.get().in(unit),
            safeValue, initialValue, centerValue, minValue, maxValue, initiallyEnabled
        );
    }

    public static ResponsiveToggleableSlider voltSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        Consumer<Voltage> voltageConsumer, Supplier<Voltage> responseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            voltageConsumer, responseSupplier, Volts,
            0.0, 0.0, 0.0, -12.0, 12.0, false
        );
    }

    public static ResponsiveToggleableSlider voltSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        Consumer<Voltage> voltageConsumer, Supplier<Voltage> leftResponseSupplier, Supplier<Voltage> rightResponseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            voltageConsumer, leftResponseSupplier, rightResponseSupplier, Volts,
            0.0, 0.0, 0.0, -12.0, 12.0, false
        );
    }

    public static ResponsiveToggleableSlider meterSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex, double minValue, double maxValue,
        Consumer<Distance> meterConsumer, Supplier<Distance> responseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            meterConsumer, responseSupplier, Meters,
            0.0, 0.0, 0.0, minValue, maxValue, false
        );
    }

    public static ResponsiveToggleableSlider meterSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex, double minValue, double maxValue,
        Consumer<Distance> meterConsumer, Supplier<Distance> leftResponseSupplier, Supplier<Distance> rightResponseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            meterConsumer, leftResponseSupplier, rightResponseSupplier, Meters,
            0.0, 0.0, 0.0, minValue, maxValue, false
        );
    }

    public static ResponsiveToggleableSlider meterPerSecondSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex, double maxValue,
        Consumer<LinearVelocity> meterPerSecondConsumer, Supplier<LinearVelocity> responseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            meterPerSecondConsumer, responseSupplier, MetersPerSecond,
            0.0, 0.0, 0.0, 0.0, maxValue, false
        );
    }

    public static ResponsiveToggleableSlider meterPerSecondSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex, double maxValue,
        Consumer<LinearVelocity> meterPerSecondConsumer, Supplier<LinearVelocity> leftResponseSupplier, Supplier<LinearVelocity> rightResponseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            meterPerSecondConsumer, leftResponseSupplier, rightResponseSupplier, MetersPerSecond,
            0.0, 0.0, 0.0, 0.0, maxValue, false
        );
    }

    public static ResponsiveToggleableSlider radianSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex,
        Consumer<Angle> radianConsumer, Supplier<Angle> responseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            radianConsumer, responseSupplier, Radians,
            0.0, 0.0, 0.0, 0.0, 2 * Math.PI, false
        );
    }

    public static ResponsiveToggleableSlider radianPerSecondSlider(
        ShuffleboardContainer container, String title,
        int columnIndex, int rowIndex, double maxValue,
        Consumer<AngularVelocity> radianConsumer, Supplier<AngularVelocity> responseSupplier
    ) {
        return unitSlider(
            container, title,
            columnIndex, rowIndex,
            radianConsumer, responseSupplier, RadiansPerSecond,
            0.0, 0.0, 0.0, 0.0, maxValue, false
        );
    }
}

package frc.lib.utility;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CumulativeDutyCycleEncoder {
    /** the encoder to wrap around */
    private final DutyCycleEncoder encoder;

    /** the maximum encoder value */
    private final double range;

    /** the value-change threshold, above which a rollover is detected and recorded */
    private final double rollOverThreshold;

    /** the value recorded the last time .update() or the constructor was called */
    private double lastValue;

    /** the number of revolutions that have accumulated */
    private int revolutions;

    // initialization
    /**
     * initializes an accumulative wrapper around a DutyCycleEncoder that detects rollovers (i.e., sudden changes in a domain-constrained value that indicate
     * the value "rolled over" the boundary) and tracks how many times that has occurred to give an accumulated position value
     * <p>
     * warning: if the encoder value changes more (or equivalently) rapidly than the maximum encoder value divided by the time between checking .get(),
     * this class will not work, as the value could roll over to the same value (or slightly more or less than) it was before, making rollovers undetectable
     * 
     * @param encoder the DutyCycleEncoder to wrap around
     * @param range the maximum encoder value
     * @param rollOverThreshold the value-change threshold, above which a rollover is detected and recorded - cannot be higher than the maximum encoder value
     */
    public CumulativeDutyCycleEncoder(DutyCycleEncoder encoder, double range, double rollOverThreshold) {
        if (rollOverThreshold > range) {
            throw new IllegalArgumentException("rollOverThreshold cannot be higher than the maximum encoder value!");
        }

        // initialization
        this.encoder = encoder;
        this.range = range;
        this.rollOverThreshold = rollOverThreshold;

        // initial state hydration
        lastValue = encoder.get();
    }

    // state modification
    /** compares the previous value to the current value to determine whether or not a rollover has occurred, and, if so, to record it */
    public void update() {
        double value = encoder.get();

        if (Math.abs(value - lastValue) > rollOverThreshold) {
            if (value > lastValue) {
                revolutions--;
            } else {
                revolutions++;
            }
        }

        lastValue = value;
    }

    // state
    /** the number of accumulated revolutions */
    public int getRevolutions() {
        return revolutions;
    }

    /** the accumulated position */
    public double get() {
        return encoder.get() + revolutions * range;
    }
}

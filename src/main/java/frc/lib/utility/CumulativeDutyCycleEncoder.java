package frc.lib.utility;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CumulativeDutyCycleEncoder {
    // hardware
    /** the encoder to wrap around */
    private final DutyCycleEncoder encoder;

    // calculation
    /** the maximum encoder value */
    private final double range;

    /** the value-change threshold, above which a rollover is detected and recorded */
    private final double rollOverThreshold;

    /** the conversion factor that the returned accumulated position is multiplied by */
    private double positionConversionFactor = 1.0;

    // state
    /** the value recorded the last time .update() or the constructor was called */
    private double lastValue;

    /** the number of revolutions that have accumulated */
    private int revolutions;

    // initialization
    /**
     * initializes an accumulative wrapper around a DutyCycleEncoder that detects rollovers (i.e., sudden changes in a domain-constrained value that indicate
     * the value "rolled over" the boundary) and tracks how many times that has occurred to give an accumulated position value
     * <p>
     * the max change parameter is used to calculate the rollover threshold by subtracting it from the range parameter, as the change for a rollover
     * equals the absolute difference between the range and the actual change, and thus, the max rollover change equals the absolute difference between
     * the range and the max change. unfortunately, <b>this value must be less than half the range</b>, as values higher could make valid changes below
     * the max change falsely recognized as rollovers if they occurred entirely within range (e.g., for range 1 and maxChange 0.6, rollOverThreshold would
     * by necessity be 0.4, making changes between 0.4 and 0.6 falsely recognized as rollovers), making the revolution count inaccurate
     * <p>
     * <b>warning</b>: if the encoder value changes by more than the range, this class <b>will not work</b>, as the value could roll over to the same value
     * (or slightly more or less than) it was before, making rollovers undetectable. furthermore, if it changes by more than half the range, it also <b>will
     * not work</b>, because rollovers with changes that high cannot be detected without introducing false positives, which make the revolution count inaccurate
     * 
     * @param encoder the DutyCycleEncoder to wrap around
     * @param range the maximum encoder value, where range > 0
     * @param maxChange the maximum change in value possible in situations where revolution accuracy is necessary, where 0 < maxChange < range / 2
     */
    public CumulativeDutyCycleEncoder(DutyCycleEncoder encoder, double range, double maxChange) {
        if (maxChange >= range / 2) {
            throw new IllegalArgumentException("maxChange cannot be greater than half the range!");
        }

        // initialization
        this.encoder = encoder;
        this.range = range;
        this.rollOverThreshold = range - maxChange;

        // initial state hydration
        lastValue = encoder.get();
    }

    // state modification
    /** compares the previous value to the current value to determine whether or not a rollover has occurred, and, if so, to record it */
    public void update() {
        // get current value
        double value = encoder.get();

        // detect and record rollover, if present
        if (Math.abs(value - lastValue) > rollOverThreshold) {
            if (value > lastValue) {
                revolutions--;
            } else {
                revolutions++;
            }
        }

        // update last value
        lastValue = value;
    }

    /** sets the position conversion factor */
    public void setPositionConversionFactor(double cf) {
        positionConversionFactor = cf;
    }

    // state
    /** the number of accumulated revolutions */
    public int getRevolutions() {
        return revolutions;
    }

    /** the accumulated position */
    public double getAccumulatedPosition() {
        double accumulatedPosition = encoder.get() + revolutions * range;
        double converted = accumulatedPosition * positionConversionFactor;

        return converted;
    }
}

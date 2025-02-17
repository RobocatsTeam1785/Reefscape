package frc.lib.utility;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Translation3dUtils {
    // for <U extends Unit>, U.of(x) always returns Measure<U>, so a cast is valid
    @SuppressWarnings("unchecked")
    public static <U extends Unit> Translation3d convertUnits(Translation3d position, U from, U to) {
        Measure<U> x = (Measure<U>)from.of(position.getX());
        Measure<U> y = (Measure<U>)from.of(position.getY());
        Measure<U> z = (Measure<U>)from.of(position.getZ());

        return new Translation3d(x.in(to), y.in(to), z.in(to));
    }
}

package frc.lib.utility;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Translation3dUtils {
    // for <U extends Unit>, U.of(x) always returns U, so a cast is valid
    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> Translation3d convertUnits(Translation3d position, U from, U to) {
        M x = (M)from.of(position.getX());
        M y = (M)from.of(position.getY());
        M z = (M)from.of(position.getZ());

        return new Translation3d(x.in(to), y.in(to), z.in(to));
    }
}

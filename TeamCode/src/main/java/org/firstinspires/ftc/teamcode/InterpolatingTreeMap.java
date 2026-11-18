package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Map;
import java.util.TreeMap;

/**
 * A TreeMap extension that can interpolate between the values of two keys
 * if the exact requested key does not exist.
 */
@Disabled
public class InterpolatingTreeMap extends TreeMap<Double, Double> {
    private static final long serialVersionUID = 1L;


    /**
     * Gets the exact value if it exists, or returns a linearly interpolated
     * value based on the nearest surrounding keys.
     *
     * @param key The key to look up or interpolate.
     * @return The interpolated value, or null if the map is empty.
     */
    @Override
    public Double get(Object key) {
        if (this.isEmpty()) {
            return null;
        }

        // Ensure the key is a Double
        if (!(key instanceof Double)) {
            throw new IllegalArgumentException("Key must be of type Double");
        }

        Double queryKey = (Double) key;

        // 1. If we have the exact key, return it immediately
        Double exactValue = super.get(queryKey);
        if (exactValue != null) {
            return exactValue;
        }

        // 2. Fetch the closest bounding entries
        Map.Entry<Double, Double> floorEntry = this.floorEntry(queryKey);
        Map.Entry<Double, Double> ceilingEntry = this.ceilingEntry(queryKey);

        // 3. Handle boundary conditions
        if (floorEntry == null) {
            return ceilingEntry.getValue(); // Out of bounds low -> return lowest element
        }
        if (ceilingEntry == null) {
            return floorEntry.getValue(); // Out of bounds high -> return highest element
        }

        // 4. Perform Linear Interpolation
        double x0 = floorEntry.getKey();
        double y0 = floorEntry.getValue();
        double x1 = ceilingEntry.getKey();
        double y1 = ceilingEntry.getValue();

        // Calculate interpolation factor t (how far queryKey is between x0 and x1)
        double t = (queryKey - x0) / (x1 - x0);

        // Linear interpolation formula: y = y0 + t * (y1 - y0)
        return y0 + t * (y1 - y0);
    }
}
package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class TunableNumber implements DoubleSupplier {
    private static final String tableKey = "/tuning";
    private final String key;

    private boolean hasDefault = false;
    private double defaultValue;

    private NetworkTableEntry ntEntry;
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    public TunableNumber(String dashboardKey) {
        this.key = dashboardKey;
    }

    public TunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    public void initDefault(double defaultValue) {
        if (hasDefault) return;

        hasDefault = true;
        this.defaultValue = defaultValue;

        // ALWAYS create the entry so it exists even outside tuning mode
        ntEntry = NetworkTableInstance.getDefault()
                .getTable(tableKey)
                .getEntry(key);

        // Only push default value to NT in tuning mode
        if (Constants.TUNING_MODE) {
            ntEntry.setDouble(defaultValue);
        }
    }

    public double get() {
        if (!hasDefault) return 0.0;

        if (Constants.TUNING_MODE) {
            return ntEntry.getDouble(defaultValue);
        } else {
            return defaultValue;
        }
    }

    public boolean hasChanged(int id) {
        double cur = get();
        Double last = lastHasChangedValues.get(id);

        if (last == null || cur != last) {
            lastHasChangedValues.put(id, cur);
            return true;
        }

        return false;
    }

    public static void ifChanged(int id, Consumer<double[]> action, TunableNumber... tunables) {
        boolean changed = Arrays.stream(tunables).anyMatch(t -> t.hasChanged(id));
        if (changed) {
            action.accept(Arrays.stream(tunables).mapToDouble(TunableNumber::get).toArray());
        }
    }

    public static void ifChanged(int id, Runnable action, TunableNumber... tunables) {
        ifChanged(id, vals -> action.run(), tunables);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}

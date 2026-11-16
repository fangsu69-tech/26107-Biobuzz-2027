package org.firstinspires.ftc.teamcode.utils;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class TelemetryUtils {
    public static boolean debugMode = false;
    public static Telemetry dsTelemetry = null;
    public static TelemetryManager dashboardTelemetry = null;

    public static void init(Telemetry opModeTelemetry) {
        dsTelemetry = opModeTelemetry;
        dashboardTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        dsTelemetry.setMsTransmissionInterval(50);
    }


    public static void addData(String key, Object value) {
        if (dsTelemetry != null) dsTelemetry.addData(key, value);
        if (dashboardTelemetry != null) dashboardTelemetry.addData(key, value);
    }



    public static void debug(String line) {
        String debugKey = "[DEBUG] " + line;
        if (debugMode) {
            if (dsTelemetry != null) dsTelemetry.addData(debugKey, "");
            if (dashboardTelemetry != null) dashboardTelemetry.debug(line);
        }
    }

    public static void debug(String key, Object value) {
        String debugKey = "[DEBUG] " + key;
        if (debugMode) {
            if (dsTelemetry != null) dsTelemetry.addData(debugKey, value);
            if (dashboardTelemetry != null) dashboardTelemetry.debug(key, value);
        }
    }


    public static void update() {
        if (dsTelemetry != null) dsTelemetry.update();
        if (dashboardTelemetry != null) dashboardTelemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryData {
    private final Telemetry telemetry;
    private ArrayList<String> dataList = new ArrayList<>();

    public TelemetryData(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
        dataList.add(caption + ": " + value.toString());
    }

    public void update() {
        telemetry.update();
        for (String data : dataList) {
            System.out.println(data);
        }

        dataList.clear();
    }
}

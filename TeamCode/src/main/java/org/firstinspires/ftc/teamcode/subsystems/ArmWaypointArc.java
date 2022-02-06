package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ArmWaypointArc {
    public static final Map<String, ArmWaypoint> waypointMap = new HashMap<String, ArmWaypoint>() {{
        put("Intake", new ArmWaypoint(0, 0, Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE));
        put("Left", new ArmWaypoint(0, 0, Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE));
        put("Right", new ArmWaypoint(0, 0, Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE));
        put("Back", new ArmWaypoint(0, 0, Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE));
    }};

    static final List<ArmWaypoint> waypointList = new ArrayList<>(Arrays.asList(
            waypointMap.get("Right"),
            waypointMap.get("Intake"),
            waypointMap.get("Left"),
            waypointMap.get("Back")
    ));

    static List<ArmWaypoint> generatePath(ArmWaypoint currentWaypoint, ArmWaypoint targetWaypoint) {
        int currentIndex = waypointList.indexOf(currentWaypoint);
        int targetIndex = waypointList.indexOf(targetWaypoint);
        if (targetIndex > currentIndex) {
            return waypointList.subList(currentIndex, targetIndex);
        } else if (currentIndex > targetIndex) {
            List<ArmWaypoint> reverseSubList = new ArrayList<>();
            for (int i = currentIndex; i >= targetIndex; i--) {
                reverseSubList.add(waypointList.get(i));
            }
            return reverseSubList;
        } else {
            return new ArrayList<>();
        }
    }
}

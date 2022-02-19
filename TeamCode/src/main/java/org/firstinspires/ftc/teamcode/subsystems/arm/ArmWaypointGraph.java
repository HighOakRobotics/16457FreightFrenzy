package org.firstinspires.ftc.teamcode.subsystems.arm;

import java.util.Arrays;
import java.util.EmptyStackException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;

public class ArmWaypointGraph {

    public static ArmWaypointGraph getInstance() {
        if (instance == null) {
            System.out.println("No instance, generating instance...");
            instance = new ArmWaypointGraph();
        }
        return instance;
    }

    private static ArmWaypointGraph instance;

    public Map<ArmWaypointName, ArmWaypoint> waypointMap;
    public Map<ArmWaypoint.ArmWaypointLocation, ArmWaypointGraph.ArmWaypointName> trackingWaypoints;

    private ArmWaypointGraph() {
        waypointMap = new HashMap<ArmWaypointName, ArmWaypoint>() {{
            // Safe travel waypoints
            put(ArmWaypointName.INTAKE_SAFE, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.FRONT,
                    0, 2 * Math.PI / 3,
                    Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.LEFT_SAFE, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.LEFT,
                    -1 * Math.PI / 2, 2 * Math.PI / 3,
                    Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.RIGHT_SAFE, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.RIGHT,
                    Math.PI / 2, 2 * Math.PI / 3,
                    Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.BACK_SAFE, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.BACK,
                    -1 * Math.PI, 2 * Math.PI / 3,
                    Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE
            ));
            // Tracking waypoints
            put(ArmWaypointName.LEFT_TRACKING, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.TRACKING,
                    ArmWaypoint.ArmWaypointLocation.LEFT,
                    -1 * Math.PI / 2, 2 * Math.PI / 3 + Math.PI / 12,
                    Arm.WristState.TRACKING_STAGING, Arm.GripperState.TARGET
            ));
            put(ArmWaypointName.RIGHT_TRACKING, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.TRACKING,
                    ArmWaypoint.ArmWaypointLocation.RIGHT,
                    Math.PI / 2, 2 * Math.PI / 3 + Math.PI / 12,
                    Arm.WristState.TRACKING_STAGING, Arm.GripperState.TARGET
            ));
            put(ArmWaypointName.BACK_TRACKING, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.TRACKING,
                    ArmWaypoint.ArmWaypointLocation.BACK,
                    -1 * Math.PI, 2 * Math.PI / 3 + Math.PI / 12,
                    Arm.WristState.TRACKING_STAGING, Arm.GripperState.TARGET
            ));
            // Intake staging waypoints
            put(ArmWaypointName.INTAKE_DOWN_UPRIGHT, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.FRONT,
                    0, 0,
                    Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.INTAKE_DOWN_CLOSED, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.FRONT,
                    0, 0,
                    Arm.WristState.HORIZONTAL, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.INTAKE_DOWN_READY, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.FRONT,
                    0, 0,
                    Arm.WristState.HORIZONTAL, Arm.GripperState.INTAKE
            ));
            // Back staging waypoints
            put(ArmWaypointName.BACK_DOWN_UPRIGHT, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.BACK,
                    -1 * Math.PI, 0,
                    Arm.WristState.UPRIGHT, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.BACK_DOWN_CLOSED, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.BACK,
                    -1 * Math.PI, 0,
                    Arm.WristState.HORIZONTAL, Arm.GripperState.CLOSE
            ));
            put(ArmWaypointName.BACK_DOWN_READY, new ArmWaypoint(
                    ArmWaypoint.ArmWaypointType.FIXED,
                    ArmWaypoint.ArmWaypointLocation.BACK,
                    -1 * Math.PI, 0,
                    Arm.WristState.HORIZONTAL, Arm.GripperState.ELEMENT
            ));
        }};
        // Make sure waypoints know their names
        applyNamesToWaypoints();
        // Safe travel waypoint links
        //// Main Arc
        waypointMap.get(ArmWaypointName.RIGHT_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.INTAKE_SAFE));
        waypointMap.get(ArmWaypointName.INTAKE_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.LEFT_SAFE));
        waypointMap.get(ArmWaypointName.LEFT_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_SAFE));
        //// Alternate Paths
        waypointMap.get(ArmWaypointName.RIGHT_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.LEFT_SAFE));
        waypointMap.get(ArmWaypointName.RIGHT_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_SAFE));
        waypointMap.get(ArmWaypointName.INTAKE_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_SAFE));
        // Tracking waypoint links
        waypointMap.get(ArmWaypointName.RIGHT_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.RIGHT_TRACKING));
        waypointMap.get(ArmWaypointName.LEFT_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.LEFT_TRACKING));
        waypointMap.get(ArmWaypointName.BACK_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_TRACKING));
        // Intake staging waypoint links
        waypointMap.get(ArmWaypointName.INTAKE_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.INTAKE_DOWN_UPRIGHT));
        waypointMap.get(ArmWaypointName.INTAKE_DOWN_UPRIGHT)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.INTAKE_DOWN_CLOSED));
        waypointMap.get(ArmWaypointName.INTAKE_DOWN_CLOSED)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.INTAKE_DOWN_READY));
        // Back staging waypoint links
        waypointMap.get(ArmWaypointName.BACK_SAFE)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_DOWN_UPRIGHT));
        waypointMap.get(ArmWaypointName.BACK_DOWN_UPRIGHT)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_DOWN_CLOSED));
        waypointMap.get(ArmWaypointName.BACK_DOWN_CLOSED)
                .addNeighbor(
                        waypointMap.get(ArmWaypointName.BACK_DOWN_READY));
        // Tracking zones
        trackingWaypoints = new HashMap<ArmWaypoint.ArmWaypointLocation, ArmWaypointGraph.ArmWaypointName>() {{
            put(ArmWaypoint.ArmWaypointLocation.RIGHT, ArmWaypointName.RIGHT_TRACKING);
            put(ArmWaypoint.ArmWaypointLocation.LEFT, ArmWaypointName.LEFT_TRACKING);
            put(ArmWaypoint.ArmWaypointLocation.BACK, ArmWaypointName.BACK_TRACKING);
        }};
    }

    public List<ArmWaypoint> generatePath(ArmWaypointName from, ArmWaypointName to) {
        System.out.println("Generating path from " + from + " to " + to);
        Stack<PathingNode> stack = new Stack<>();
        Set<ArmWaypoint> visited = new HashSet<>();
        stack.push(new PathingNode(waypointMap.get(from), null));
        visited.add(waypointMap.get(from));
        ArmWaypoint target = waypointMap.get(to);
        while (!stack.empty()) {
            PathingNode hand = stack.pop();
            if (hand.getWaypoint().equals(target))
                return backtrackPathingNode(hand);
            for (ArmWaypoint armWaypoint : hand.getWaypoint().getNeighbors()) {
                if (!visited.contains(armWaypoint)) {
                    stack.push(new PathingNode(
                            armWaypoint,
                            hand
                    ));
                    visited.add(armWaypoint);
                }
            }
        }
        throw new EmptyStackException();
    }

    private List<ArmWaypoint> backtrackPathingNode(PathingNode endpoint) {
        List<ArmWaypoint> result = new LinkedList<>();
        PathingNode hand = endpoint;
        result.add(0, hand.waypoint);
        while (hand.getParent() != null) {
            result.add(0, hand.getParent().getWaypoint());
            hand = hand.getParent();
        }
        return result;
    }

    private static class PathingNode {
        final ArmWaypoint waypoint;
        final PathingNode parent;

        public PathingNode(ArmWaypoint waypoint, PathingNode parent) {
            this.waypoint = waypoint;
            this.parent = parent;
        }

        public ArmWaypoint getWaypoint() {
            return waypoint;
        }

        public PathingNode getParent() {
            return parent;
        }
    }

    public void applyNamesToWaypoints() {
        for (Map.Entry<ArmWaypointGraph.ArmWaypointName, ArmWaypoint> entry : waypointMap.entrySet()) {
            entry.getValue().setName(entry.getKey());
        }
    }

    public enum ArmWaypointName {
        INTAKE_SAFE, LEFT_SAFE, RIGHT_SAFE, BACK_SAFE,
        LEFT_TRACKING, RIGHT_TRACKING, BACK_TRACKING,
        INTAKE_DOWN_UPRIGHT, INTAKE_DOWN_CLOSED, INTAKE_DOWN_READY,
        BACK_DOWN_UPRIGHT, BACK_DOWN_CLOSED, BACK_DOWN_READY
    }
}

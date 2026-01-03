// File: org/firstinspires/ftc/teamcode/pedroPathing/PoseStorage.java
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap; // <-- Import HardwareMap
import com.qualcomm.robotcore.util.RobotLog; // <-- Import RobotLog for cleaner code

import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;

/**
 * A utility class to save the robot's last known pose to a file
 * and load it back. This allows for pose persistence between OpModes.
 */
public class PoseStorage {

    // The name of the file where the pose will be stored.
    private static final String FILE_NAME = "robot_pose.txt";

    /**
     * Saves the given Pose to a file.
     * The pose is stored as three comma-separated values: x,y,heading.
     *
     * @param hardwareMap The OpMode's hardwareMap, used to get file context.
     * @param pose The robot's pose to save.
     */
    public static void savePoseToFile(HardwareMap hardwareMap, Pose pose) {
        // --- THIS IS THE FIX ---
        // Get the directory for app files using the hardwareMap's context
        File file = new File(hardwareMap.appContext.getFilesDir(), FILE_NAME);

        try (PrintWriter writer = new PrintWriter(file)) {
            // Write the pose components, separated by commas
            writer.println(pose.getX() + "," + pose.getY() + "," + pose.getHeading());
        } catch (Exception e) {
            // Log the error if something goes wrong. This is important for debugging!
            RobotLog.e("PoseStorage - Error saving pose: " + e.getMessage());
        }
    }

    /**
     * Loads the last saved pose from the file.
     * If the file doesn't exist or is invalid, it returns a default (0,0,0) pose.
     *
     * @param hardwareMap The OpMode's hardwareMap, used to get file context.
     * @return The loaded Pose object.
     */
    public static Pose loadPoseFromFile(HardwareMap hardwareMap) {
        // --- THIS IS THE FIX ---
        File file = new File(hardwareMap.appContext.getFilesDir(), FILE_NAME);

        try (Scanner scanner = new Scanner(file)) {
            if (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] parts = line.split(",");
                if (parts.length == 3) {
                    double x = Double.parseDouble(parts[0]);
                    double y = Double.parseDouble(parts[1]);
                    double heading = Double.parseDouble(parts[2]);
                    return new Pose(x, y, heading);
                }
            }
        } catch (Exception e) {
            // Log the error if the file isn't found or is corrupt.
            RobotLog.w("PoseStorage - No pose file found or file is invalid. Returning default pose.");
        }
        // Return a default pose if loading fails for any reason
        return new Pose(0, 0, 0);
    }
}

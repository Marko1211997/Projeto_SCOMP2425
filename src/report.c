#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h> 

#include "include/report.h"
#include "include/utils.h" // For count_lines

void generate_report() {
    FILE* report_file_handle = fopen(REPORT_FILENAME, "w"); // Renamed
    if (!report_file_handle) {
        perror("Error creating report file");
        return;
    }
    
    time_t current_time_val = time(NULL);
    char time_str_buffer[100]; // Renamed
    strftime(time_str_buffer, sizeof(time_str_buffer), "%Y-%m-%d %H:%M:%S", localtime(&current_time_val));
    
    fprintf(report_file_handle, "=======================================================\n");
    fprintf(report_file_handle, "             DRONE FIGURE SIMULATION REPORT            \n");
    fprintf(report_file_handle, "=======================================================\n\n");
    fprintf(report_file_handle, "Generated: %s\n", time_str_buffer);
    fprintf(report_file_handle, "Figure File: %s\n\n", figure_filename);
    
    fprintf(report_file_handle, "-------------------------------------------------------\n");
    fprintf(report_file_handle, "SUMMARY\n\n");
    fprintf(report_file_handle, "Total Number of Drones: %d\n", drone_count);
    fprintf(report_file_handle, "Simulation Result: %s\n\n", collision_detected ? "FAILED" : "PASSED");
    
    fprintf(report_file_handle, "-------------------------------------------------------\n");
    fprintf(report_file_handle, "DRONE's STATUS\n\n");
    
    for (int i = 0; i < drone_count; i++) {
        char script_file_name_buffer[256]; // Renamed
        sprintf(script_file_name_buffer, "drone_%d_script.txt", i);

        bool drone_involved_in_collision = false; // Renamed
        for (int j = 0; j < collision_count; j++) {
            if (collisions[j].drone1_id == i || collisions[j].drone2_id == i) {
                drone_involved_in_collision = true;
                break;
            }
        }

        int reported_steps = 0; // Renamed
        // Logic for reported_steps based on original US261.c
        if (drones[i].active && !max_col) { // Completed successfully (not terminated by max_col, and still active)
            reported_steps = count_lines(script_file_name_buffer);
            if (reported_steps == -1) reported_steps = 0; 
        } else if (drone_involved_in_collision) { // Terminated due to a collision it was part of
            for(int j = 0; j < collision_count; j++) {
                if (collisions[j].drone1_id == i || collisions[j].drone2_id == i) {
                    reported_steps = (int)collisions[j].time; 
                    break; 
                }
            }
        } else { // Incomplete, or terminated due to max_col globally, or SIGINT
             // If max_col is true and drone is !active, it was terminated.
             // If simulation_running became false (e.g. SIGINT), step reflects progress.
             reported_steps = step > 0 ? step -1 : 0; // Global step when simulation stopped or drone became inactive
        }
        
        fprintf(report_file_handle, "Drone %d:\n", i);
        fprintf(report_file_handle, "  Script: %s\n", script_file_name_buffer);
        fprintf(report_file_handle, "  Total Steps: %d\n", reported_steps);
        
        const char* drone_status_str; // Renamed
        if (drones[i].active && !max_col && !drone_involved_in_collision) {
            // If drone is active, not max_col scenario, and wasn't in a collision, it completed.
            // The original `drones[i].active && !max_col` was for "Completed".
            // Adding `!drone_involved_in_collision` for clarity, though `drones[i].active` should be false if it was.
            drone_status_str = "Completed Successfully!";
        } else if (drone_involved_in_collision) {
            drone_status_str = "Terminated (Collision)!";
        } else if (max_col && !drones[i].active) { // Terminated because max_col reached
             drone_status_str = "Terminated (Max Collisions Reached)!";
        } else if (!drones[i].active) { // Terminated for other reasons or general incomplete
            drone_status_str = "Terminated or Incomplete!";
        }
         else { // Fallback: still active but max_col might be true (e.g. other drones hit max_col)
            drone_status_str = "Incomplete!";
        }
        
        fprintf(report_file_handle, "  Status: %s\n", drone_status_str);
        fprintf(report_file_handle, "  Final Position: (%.2f, %.2f, %.2f)\n\n", 
                drones[i].x, drones[i].y, drones[i].z);
    }
    
    if (collision_detected) {
        fprintf(report_file_handle, "-------------------------------------------------------\n");
        fprintf(report_file_handle, "COLLISION(S) DETAILS\n\n");
        fprintf(report_file_handle, "Total Number of Collisions: %d\n\n", collision_count);
        
        for (int i = 0; i < collision_count; i++) {
            fprintf(report_file_handle, "Collision %d:\n", i + 1);
            fprintf(report_file_handle, "  Drones Involved: %d and %d\n", 
                    collisions[i].drone1_id, collisions[i].drone2_id);
            fprintf(report_file_handle, "  Time: %.2f seconds\n", collisions[i].time);        
            fprintf(report_file_handle, "  Distance between them: %.2f meters\n", collisions[i].distance);
            fprintf(report_file_handle, "  Drone %d Position: (%.2f, %.2f, %.2f)\n", 
                    collisions[i].drone1_id, 
                    collisions[i].x1, collisions[i].y1, collisions[i].z1);
            fprintf(report_file_handle, "  Drone %d Position: (%.2f, %.2f, %.2f)\n\n", 
                    collisions[i].drone2_id, 
                    collisions[i].x2, collisions[i].y2, collisions[i].z2);
        }
    }
    
    fprintf(report_file_handle, "-------------------------------------------------------\n");
    fprintf(report_file_handle, "RECOMMENDATIONS\n\n");
    if (collision_detected) {
        fprintf(report_file_handle, "The figure is NOT safe to use.\nPlease modify the drone paths to avoid collisions.\n");
        fprintf(report_file_handle, "Consider adjusting the paths of the following drones:\n");
        for (int i = 0; i < collision_count; i++) {
            fprintf(report_file_handle, "- Drones %d and %d (collided at time %.2f)\n",
                    collisions[i].drone1_id, collisions[i].drone2_id, collisions[i].time);
        }
    } else {
        fprintf(report_file_handle, "The figure is safe to use.\nAll drones completed their paths without collisions.\n");
    }
    
    fclose(report_file_handle);
    printf("Simulation report generated: %s\n", REPORT_FILENAME);
}
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h> // For waitpid in terminate_drone

#include "include/drone.h"
#include "include/utils.h" // For signal_handler

void drone_process(Drone* drone_ptr, const char* script_file_path) { // Renamed params
    signal(SIGTERM, signal_handler); 

    // Child process: Close the read end of the pipe it won't use.
    // The `drone_ptr` here is a copy of the parent's struct member.
    // Its `pipe_read` and `pipe_write` are the correct FDs for this drone's pipe.
    close(drone_ptr->pipe_read); 
    
    FILE* file = fopen(script_file_path, "r");
    if (!file) {
        perror("Error opening drone script file");
        close(drone_ptr->pipe_write); // Close write end before exiting
        exit(EXIT_FAILURE);
    }
    
    char line[256];
    double script_time, script_x, script_y, script_z; // Renamed local vars
    
    Position current_pos_drone = {drone_ptr->x, drone_ptr->y, drone_ptr->z, 0.0}; // Renamed
    
    if (write(drone_ptr->pipe_write, &current_pos_drone, sizeof(Position)) == -1) {
        // perror("Drone: Error writing initial position to pipe");
        fclose(file);
        close(drone_ptr->pipe_write);
        exit(EXIT_FAILURE);
    }
    
    while (simulation_running && fgets(line, sizeof(line), file)) {
        if (!simulation_running) break; 

        if (sscanf(line, "%lf %lf %lf %lf", &script_time, &script_x, &script_y, &script_z) == 4) {
            current_pos_drone.x = script_x;
            current_pos_drone.y = script_y;
            current_pos_drone.z = script_z;
            current_pos_drone.time = script_time;
            
            if (write(drone_ptr->pipe_write, &current_pos_drone, sizeof(Position)) == -1) {
                // perror("Drone: Error writing position to pipe");
                break; 
            }
            
            // Original sleep logic from US261.c:
            // This drone sleeps for an absolute duration specified in the script line's time field.
            usleep((useconds_t)(script_time * 1000000)); 
        }
        if (!simulation_running) break; 
    }
    
    fclose(file);
    close(drone_ptr->pipe_write); // Signal EOF to parent
    // Drone process exits.
}

void terminate_drone(int drone_id_to_terminate) { // Renamed param
    if (drone_id_to_terminate < 0 || drone_id_to_terminate >= drone_count) {
        return;
    }
    if (!drones[drone_id_to_terminate].active) {
        return;
    }
    printf("Terminating drone %d \n", drone_id_to_terminate);
    
    if (drones[drone_id_to_terminate].pid > 0) {
        kill(drones[drone_id_to_terminate].pid, SIGTERM);
        waitpid(drones[drone_id_to_terminate].pid, NULL, 0); 
        drones[drone_id_to_terminate].pid = 0; 
    }

    drones[drone_id_to_terminate].active = false;
    
    // Parent closes its read end for this drone's pipe
    if (drones[drone_id_to_terminate].pipe_read != -1) {
        close(drones[drone_id_to_terminate].pipe_read);
        drones[drone_id_to_terminate].pipe_read = -1;
    }
    // The write end (drones[drone_id_to_terminate].pipe_write) was the child's write end.
    // Parent already closed its copy of this FD after fork.
    // The original code also closed pipe_write here, which is harmless if already closed or invalid.
}

void terminate_drone_all() {
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].active) { 
            terminate_drone(i);
        }
    }
}
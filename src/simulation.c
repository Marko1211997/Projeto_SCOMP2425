#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/wait.h>
#include <math.h>
// No need for <signal.h> here if signal_handler is in utils and drone_process handles its own
// #include <signal.h> 

#include "include/simulation.h"
#include "include/drone.h" 
#include "include/utils.h" 

void initialize_simulation(const char* figure_file_path) { // Renamed parameter
    FILE* file = fopen(figure_file_path, "r");
    if (!file) {
        perror("Error opening figure file!");
        exit(EXIT_FAILURE);
    }
    
    char line[256];
    char script_file_buffer[256]; // Renamed buffer
    double x, y, z;
    
    while (fgets(line, sizeof(line), file) && drone_count < MAX_DRONES) {
        if (sscanf(line, "%s %lf %lf %lf", script_file_buffer, &x, &y, &z) == 4) {
            drones[drone_count].id = drone_count;
            drones[drone_count].x = x;
            drones[drone_count].y = y;
            drones[drone_count].z = z;
            drones[drone_count].active = true;  

            int pipe_fd[2];
            if (pipe(pipe_fd) == -1) {
                perror("Pipe creation failed");
                exit(EXIT_FAILURE);
            }
            
            drones[drone_count].pipe_read = pipe_fd[0];
            drones[drone_count].pipe_write = pipe_fd[1];
            
            int nL = count_lines(script_file_buffer);
            if (nL > nlMax) {
                nlMax = nL;
            }
             if (nL == -1) { // Error reading script file for line count
                fprintf(stderr, "Warning: Could not count lines for script %s\n", script_file_buffer);
            }
            drone_count++;
        }
    }
    
    fclose(file);
    if (drone_count == 0) {
        fprintf(stderr, "Error: No drones found in figure file!\n");
        exit(EXIT_FAILURE);
    } else {
        printf("Initialized %d drones for simulation\n", drone_count);
    }
}

void start_simulation() {
    printf("Starting simulation with %d drones\n", drone_count);
    
    for (int i = 0; i < drone_count; i++) {
        pid_t pid = fork();
        
        if (pid == -1) {
            perror("Fork failed!");
            exit(EXIT_FAILURE);
        } else if (pid == 0) {
            // Child process (drone)
            char script_file_for_drone[256];
            sprintf(script_file_for_drone, "drone_%d_script.txt", drones[i].id);
            // Note: drones[i] is a copy of the parent's memory.
            // The pipe descriptors are correct.
            drone_process(&drones[i], script_file_for_drone);
            exit(EXIT_SUCCESS);
        } else {
            // Parent process
            drones[i].pid = pid;
            close(drones[i].pipe_write); // Parent closes its copy of the write end of the pipe
            printf("Started drone %d with PID %d\n", i, pid);
        }
    }
    printf("\n");
   
    while (simulation_running && step < MAX_STEPS && step < nlMax + 1) {
        double current_loop_time = 0.0; // Time for the current step, taken from a drone
        bool any_active_in_loop = false;

        for (int i = 0; i < drone_count; i++) {
            if (!drones[i].active) {
                continue;
            }
            any_active_in_loop = true;
            Position pos;
            ssize_t bytes_read = read(drones[i].pipe_read, &pos, sizeof(Position));
            
            if (bytes_read == sizeof(Position)) {
                drones[i].x = pos.x;
                drones[i].y = pos.y;
                drones[i].z = pos.z;
                printf("Drone %d at position (%.2f, %.2f, %.2f) at time %.2f\n", 
                       i, pos.x, pos.y, pos.z, pos.time);
                current_loop_time = pos.time; // Use time from the last drone read for collision check      
            } else if (bytes_read == 0) {
                // Pipe closed by drone (likely finished its script or terminated)
                // The drone process should exit, and waitpid in cleanup_simulation will reap it.
                // No explicit action here, active flag managed by termination logic.
            } else if (bytes_read == -1) {
                // Can occur if pipe is non-blocking and no data, or actual error.
                // Original code implies blocking read, so this would be an error or pipe closed.
                // perror("Parent: Read from pipe failed"); 
            }
        }

        if (!any_active_in_loop && step > 0) { 
             if (!check_active_drones()) { 
                printf("All drones have completed or been terminated\n");
                simulation_running = false; 
                break;
             }
        }
        
        printf("\n");
        if (simulation_running) { // Check if simulation is still running before collision check
             check_collisions(current_loop_time);
        }
        
        if (!simulation_running) break; // If collision check stopped simulation

        step++;
        usleep(100000);
    }
    
    if (!collision_detected && step >= nlMax +1 ) {
      
        for (int i = 0; i < drone_count; i++) {
            
             if (simulation_running) { // Check if simulation wasn't stopped by signal
               
             }
        }
    }
    
    printf("Simulation completed after %d steps\n", step > 0 ? step -1 : 0);
}

void check_collisions(double time_check) { // Renamed parameter
    bool बिल_terminate[MAX_DRONES] = {false}; // Using a different name for local array
    bool new_collision_this_step = false;

    for (int i = 0; i < drone_count; i++) {
        if (!drones[i].active) {
            continue;
        }
        for (int j = i + 1; j < drone_count; j++) {
            if (!drones[j].active) { // Also check drone j
                continue;
            }
            double dx = drones[i].x - drones[j].x;
            double dy = drones[i].y - drones[j].y;
            double dz = drones[i].z - drones[j].z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
            if (distance < COLLISION_THRESHOLD) {
                if (collision_count < MAX_COLLISIONS) {
                    printf("COLLISION ALERT: Drones %d and %d are too close (%.2f meters)!\n\n", 
                           i, j, distance);
                    
                    collisions[collision_count].drone1_id = i;
                    collisions[collision_count].drone2_id = j;
                    collisions[collision_count].distance = distance;
                    collisions[collision_count].time = time_check; 
                    collisions[collision_count].x1 = drones[i].x;
                    collisions[collision_count].y1 = drones[i].y;
                    collisions[collision_count].z1 = drones[i].z;
                    collisions[collision_count].x2 = drones[j].x;
                    collisions[collision_count].y2 = drones[j].y;
                    collisions[collision_count].z2 = drones[j].z;
                    collision_count++;
                                    
                    collision_detected = true;
                    new_collision_this_step = true;

                    बिल_terminate[i] = true;
                    बिल_terminate[j] = true; 
                } else if (!max_col) { 
                     // This message might be printed multiple times if collisions continue after MAX_COLLISIONS is reached
                     // but before max_col is set.
                }
            }
        }
    }

    for (int i = 0; i < drone_count; i++) {
        if (बिल_terminate[i]) {
            terminate_drone(i);
        }
    }
    
    if (new_collision_this_step) {
        printf("\n");
    }

    if (collision_count >= MAX_COLLISIONS && !max_col) { // Check !max_col to do this once
        printf("Maximum number of collisions reached, stopping simulation!\n");
        max_col = true;
        simulation_running = false;
        terminate_drone_all();
    }
}

bool check_active_drones() {
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].active) {
            return true;
        }
    }
    return false;
}

void cleanup_simulation() {
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].pid > 0) { 
            int status;
            pid_t result = waitpid(drones[i].pid, &status, WNOHANG);
            if (result == 0) { // Process is still running
                kill(drones[i].pid, SIGTERM);
            }
            // No else needed, if result is > 0, already terminated. If -1, error.
        }
        if (drones[i].pipe_read != -1) { // Parent's read end
             close(drones[i].pipe_read);
             drones[i].pipe_read = -1; 
        }
        // drones[i].pipe_write was parent's copy of child's write end, closed after fork.
    }
    
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].pid > 0) { // If PID was set and not yet reaped by specific terminate_drone
            waitpid(drones[i].pid, NULL, 0); 
            drones[i].pid = 0; 
        }
    }
    
    printf("Simulation cleanup complete!\n");
}
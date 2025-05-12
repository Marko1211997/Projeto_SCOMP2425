#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define MAX_DRONES 100
#define MAX_STEPS 1000
#define MAX_COLLISIONS 5
#define COLLISION_THRESHOLD 1.0  // Minimum distance between drones (in meters)
#define REPORT_FILENAME "simulation_report.txt"

typedef struct {
    int id;
    double x, y, z;  // 3D coordinates
    pid_t pid;       // Process ID of the drone
    int pipe_read;   // Read end of pipe
    int pipe_write;  // Write end of pipe
    bool active;     // Flag to indicate if the drone is still active (not terminated due to collision)
} Drone;

typedef struct {
    double x, y, z;
    double time;     // Time in seconds
} Position;

typedef struct {
    int drone1_id;
    int drone2_id;
    double distance;
    double time;
    double x1, y1, z1;  // Position of drone 1
    double x2, y2, z2;  // Position of drone 2
} Collision;

// Global variables
Drone drones[MAX_DRONES];
int drone_count = 0;
bool simulation_running = true;
int nlMax = 0;
bool collision_detected = false;
Collision collisions[MAX_COLLISIONS];
int collision_count = 0;
char figure_filename[256];

// Function prototypes
void initialize_simulation(const char* figure_file);
void start_simulation();
void drone_process(Drone* drone, const char* script_file);
void check_collisions();
void cleanup_simulation();
void signal_handler(int signum);
int count_lines(const char* filename);
void generate_report();
void terminate_drone();
bool check_active_drones();

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: %s <figure_file>\n", argv[0]);
        return 1;
    }

    // Store the figure filename for the report
    strncpy(figure_filename, argv[1], sizeof(figure_filename) - 1);
    figure_filename[sizeof(figure_filename) - 1] = '\0';

    // Set up signal handler for graceful termination
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize the simulation with the figure file
    initialize_simulation(argv[1]);
    
    // Start the simulation
    start_simulation();
    
    // Generate the simulation report
    generate_report();
    
    // Clean up resources
    cleanup_simulation();
    
    return 0;
}

void initialize_simulation(const char* figure_file) {
    FILE* file = fopen(figure_file, "r");
    if (!file) {
        perror("Error opening figure file");
        exit(EXIT_FAILURE);
    }
    
    char line[256];
    char script_file[256];
    double x, y, z;
    // Read drone initial positions and script files from the figure file
    while (fgets(line, sizeof(line), file) && drone_count < MAX_DRONES) {
        if (sscanf(line, "%s %lf %lf %lf", script_file, &x, &y, &z) == 4) {
            drones[drone_count].id = drone_count;
            drones[drone_count].x = x;
            drones[drone_count].y = y;
            drones[drone_count].z = z;
            drones[drone_count].active = true;  

            // Create pipe for communication
            int pipe_fd[2];
            if (pipe(pipe_fd) == -1) {
                perror("Pipe creation failed");
                exit(EXIT_FAILURE);
            }
            
            drones[drone_count].pipe_read = pipe_fd[0];
            drones[drone_count].pipe_write = pipe_fd[1];
            
            drone_count++;
        }
        int nL = count_lines(script_file);
        if (nL >= nlMax){
            nlMax = nL;
        } 
    }
    
    fclose(file);
    if (drone_count == 0) {
        fprintf(stderr, "Error: No drones found in figure file.\n");
        exit(EXIT_FAILURE);
    } else printf("Initialized %d drones for simulation\n", drone_count);

}

void start_simulation() {
    printf("Starting simulation with %d drones\n", drone_count);
    
    // Fork a process for each drone
    for (int i = 0; i < drone_count; i++) {
        pid_t pid = fork();
        
        if (pid == -1) {
            perror("Fork failed");
            exit(EXIT_FAILURE);
        } else if (pid == 0) {
            // Child process (drone)
            char script_file[256];
            sprintf(script_file, "drone_%d_script.txt", drones[i].id);
            drone_process(&drones[i], script_file);
            exit(EXIT_SUCCESS);
        } else {
            // Parent process
            drones[i].pid = pid;
            printf("Started drone %d with PID %d\n", i, pid);
        }
    }
    printf("\n");
    // Main simulation loop
    int step = 0;
    while (simulation_running && step < MAX_STEPS && step < nlMax+1) {
        // Read positions from all drones
        double time = 0.0;
        bool any_active = false;

        for (int i = 0; i < drone_count; i++) {
            if (!drones[i].active) {
                continue;  // Skip inactive drones
            }

            any_active = true;
            Position pos;
            ssize_t bytes_read = read(drones[i].pipe_read, &pos, sizeof(Position));
            
            if (bytes_read == sizeof(Position)) {
                drones[i].x = pos.x;
                drones[i].y = pos.y;
                drones[i].z = pos.z;
                printf("Drone %d at position (%.2f, %.2f, %.2f) at time %.2f\n", 
                       i, pos.x, pos.y, pos.z, pos.time);
                time = pos.time;       
            }
                
        }
        if (!any_active) {
            printf("All drones have completed or been terminated\n");
            break;
        }
        printf("\n");
        // Check for collisions
        check_collisions(time);
        step++;
        usleep(100000);  // Sleep for 100ms between steps
    }
    
    // If we reached the end of all scripts without collisions, mark all drones as completed
    if (!collision_detected && step >= nlMax) {
        for (int i = 0; i < drone_count; i++) {
            drones[i].active = true;
        }
    }
    
    printf("Simulation completed after %d steps\n", step-1);
}

void drone_process(Drone* drone, const char* script_file) {

    // Set up signal handler for termination
    signal(SIGTERM, signal_handler);

    // Close the read end of the pipe in the drone process
    close(drone->pipe_read);
    
    FILE* file = fopen(script_file, "r");
    if (!file) {
        perror("Error opening drone script file");
        exit(EXIT_FAILURE);
    }
    
    char line[256];
    double time, x, y, z;
    
    // Initial position
    Position current_pos = {drone->x, drone->y, drone->z, 0.0};
    
    // Send initial position to main process
    write(drone->pipe_write, &current_pos, sizeof(Position));
    
    // Execute movement script
    while (fgets(line, sizeof(line), file) && simulation_running) {
        if (sscanf(line, "%lf %lf %lf %lf", &time, &x, &y, &z) == 4) {
            // Update position
            current_pos.x = x;
            current_pos.y = y;
            current_pos.z = z;
            current_pos.time = time;
            
            // Send position to main process
            write(drone->pipe_write, &current_pos, sizeof(Position));
            
            // Sleep to simulate the passage of time
            usleep((int)(time * 1000000));
        }
    }
    
    fclose(file);
    close(drone->pipe_write);
}

void check_collisions(double time) {
    // First, identify all collisions without terminating any drones
    bool will_terminate[MAX_DRONES] = {false};

    for (int i = 0; i < drone_count; i++) {
        if(!drones[i].active) {
            continue;  // Skip inactive drones
        }
        for (int j = i + 1; j < drone_count; j++) {
            // Calculate distance between drones
            if (!drones[i].active || !drones[j].active) {
                continue;  // Skip inactive drones
            }
            double dx = drones[i].x - drones[j].x;
            double dy = drones[i].y - drones[j].y;
            double dz = drones[i].z - drones[j].z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
            if (distance < COLLISION_THRESHOLD) {
                printf("COLLISION ALERT: Drones %d and %d are too close (%.2f meters)!\n\n", 
                       i, j, distance);
                
                // Record the collision if we haven't reached the maximum
                if (collision_count < MAX_COLLISIONS) {
                    collisions[collision_count].drone1_id = i;
                    collisions[collision_count].drone2_id = j;
                    collisions[collision_count].distance = distance;
                    collisions[collision_count].time = time; 
                    collisions[collision_count].x1 = drones[i].x;
                    collisions[collision_count].y1 = drones[i].y;
                    collisions[collision_count].z1 = drones[i].z;
                    collisions[collision_count].x2 = drones[j].x;
                    collisions[collision_count].y2 = drones[j].y;
                    collisions[collision_count].z2 = drones[j].z;
                    collision_count++;
                }
                                
                collision_detected = true;

                will_terminate[i] = true;
                will_terminate[j] = true; 

            }
        }
    }
    for (int i = 0; i < drone_count; i++) {
        if (will_terminate[i]) {
            terminate_drone(i);
        }
    }
    
    if (collision_detected) {
        printf("\n");
    }
}

bool check_active_drones() {
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].active) {
            return true;  // At least one drone is still active
        }
    }
    return false;  // No active drones left
}

void cleanup_simulation() {
    // Send termination signal to all drone processes
    for (int i = 0; i < drone_count; i++) {
        kill(drones[i].pid, SIGTERM);
        close(drones[i].pipe_read);
        close(drones[i].pipe_write);
    }
    
    // Wait for all child processes to terminate
    for (int i = 0; i < drone_count; i++) {
        waitpid(drones[i].pid, NULL, 0);
    }
    
    printf("Simulation cleanup complete\n");
}

void signal_handler(int signum) {
    printf("Received signal %d, terminating simulation...\n", signum);
    simulation_running = false;
}

int count_lines(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return -1;
    }
    
    int count = 0;
    char buffer[1024]; // Adjust buffer size as needed
    
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        count++;
    }
    
    fclose(file);
    return count;
}

void terminate_drone(int drone_id) {
    if (drone_id < 0 || drone_id >= drone_count) {
        return;  // Invalid drone ID
    }
    if (!drones[drone_id].active) {
        return;  // Drone already inactive
    }
    printf("Terminating drone %d due to collision\n", drone_id);
    
    // Send termination signal to the drone process
    kill(drones[drone_id].pid, SIGTERM);

    // Mark the drone as inactive and not completed
    drones[drone_id].active = false;
    
    // Wait for the process to terminate
    waitpid(drones[drone_id].pid, NULL, 0);
    
    // Close the pipe
    close(drones[drone_id].pipe_read);
    close(drones[drone_id].pipe_write);
}

void generate_report() {
    FILE* report_file = fopen(REPORT_FILENAME, "w");
    if (!report_file) {
        perror("Error creating report file");
        return;
    }
    
    // Get current time for the report
    time_t current_time = time(NULL);
    char time_str[100];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&current_time));
    
    // Write report header
    fprintf(report_file, "=======================================================\n");
    fprintf(report_file, "             DRONE FIGURE SIMULATION REPORT            \n");
    fprintf(report_file, "=======================================================\n\n");
    fprintf(report_file, "Generated: %s\n", time_str);
    fprintf(report_file, "Figure File: %s\n\n", figure_filename);
    
    // Write summary information
    fprintf(report_file, "SUMMARY\n");
    fprintf(report_file, "-------\n");
    fprintf(report_file, "Total Drones: %d\n", drone_count);
    fprintf(report_file, "Validation Result: %s\n\n", collision_detected ? "FAILED" : "PASSED");
    
    // Write drone status information
    fprintf(report_file, "DRONE STATUS\n");
    fprintf(report_file, "-----------\n");
    for (int i = 0; i < drone_count; i++) {
        char script_file[256];
        sprintf(script_file, "drone_%d_script.txt", i);
        int steps = 0;
        if (drones[i].active) {
            // Count the number of steps in the script file
            steps = count_lines(script_file);
        } else {
            for(int j = 0; j < collision_count; j++) {
                if (collisions[j].drone1_id == i || collisions[j].drone2_id == i) {
                    steps = collisions[j].time;  // Use the time of the collision as the last step
                }
            }
        }
        
        fprintf(report_file, "Drone %d:\n", i);
        fprintf(report_file, "  Script: %s\n", script_file);
        fprintf(report_file, "  Total Steps: %d\n", steps);
        
        // Check if this drone was involved in any collision
        bool involved_in_collision = false;
        for (int j = 0; j < collision_count; j++) {
            if (collisions[j].drone1_id == i || collisions[j].drone2_id == i) {
                involved_in_collision = true;
                break;
            }
        }
        
        // Determine the status message
        const char* status;
        if (drones[i].active) {
            status = "Completed Successfully";
        } else if (involved_in_collision) {
            status = "Terminated (Collision)";
        } else {
            status = "Incomplete";
        }
        
        fprintf(report_file, "  Status: %s\n", status);
        fprintf(report_file, "  Final Position: (%.2f, %.2f, %.2f)\n\n", 
                drones[i].x, drones[i].y, drones[i].z);
    }
    
    // Write collision information if any
    if (collision_detected) {
        fprintf(report_file, "COLLISION DETAILS\n");
        fprintf(report_file, "----------------\n");
        fprintf(report_file, "Total Collisions: %d\n\n", collision_count);
        
        for (int i = 0; i < collision_count; i++) {
            fprintf(report_file, "Collision %d:\n", i + 1);
            fprintf(report_file, "  Drones Involved: %d and %d\n", 
                    collisions[i].drone1_id, collisions[i].drone2_id);
            fprintf(report_file, "  Time: %.2f seconds\n", collisions[i].time);        
            fprintf(report_file, "  Distance: %.2f meters\n", collisions[i].distance);
            fprintf(report_file, "  Drone %d Position: (%.2f, %.2f, %.2f)\n", 
                    collisions[i].drone1_id, 
                    collisions[i].x1, collisions[i].y1, collisions[i].z1);
            fprintf(report_file, "  Drone %d Position: (%.2f, %.2f, %.2f)\n\n", 
                    collisions[i].drone2_id, 
                    collisions[i].x2, collisions[i].y2, collisions[i].z2);
        }
    }
    
    // Write recommendations
    fprintf(report_file, "RECOMMENDATIONS\n");
    fprintf(report_file, "--------------\n");
    if (collision_detected) {
        fprintf(report_file, "The figure is NOT safe to use. Please modify the drone paths to avoid collisions.\n");
        
        // List all collisions for recommendations
        fprintf(report_file, "Consider adjusting the paths of the following drones:\n");
        for (int i = 0; i < collision_count; i++) {
            fprintf(report_file, "- Drones %d and %d (collided at time %.2f)\n",
                    collisions[i].drone1_id, collisions[i].drone2_id, collisions[i].time);
        }
    } else {
        fprintf(report_file, "The figure is safe to use. All drones completed their paths without collisions.\n");
    }
    
    fclose(report_file);
    printf("Simulation report generated: %s\n", REPORT_FILENAME);
}
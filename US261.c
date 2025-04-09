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
#define COLLISION_THRESHOLD 1.0  // Minimum distance between drones (in meters)

typedef struct {
    int id;
    double x, y, z;  // 3D coordinates
    pid_t pid;       // Process ID of the drone
    int pipe_read;   // Read end of pipe
    int pipe_write;  // Write end of pipe
} Drone;

typedef struct {
    double x, y, z;
    double time;     // Time in seconds
} Position;

// Global variables
Drone drones[MAX_DRONES];
int drone_count = 0;
bool simulation_running = true;

// Function prototypes
void initialize_simulation(const char* figure_file);
void start_simulation();
void drone_process(Drone* drone, const char* script_file);
void check_collisions();
void cleanup_simulation();
void signal_handler(int signum);

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: %s <figure_file>\n", argv[0]);
        return 1;
    }

    // Set up signal handler for graceful termination
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize the simulation with the figure file
    initialize_simulation(argv[1]);
    
    // Start the simulation
    start_simulation();
    
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
    }
    
    fclose(file);
    printf("Initialized %d drones for simulation\n", drone_count);
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
    
    // Main simulation loop
    int step = 0;
    while (simulation_running && step < MAX_STEPS) {
        // Read positions from all drones
        for (int i = 0; i < drone_count; i++) {
            Position pos;
            ssize_t bytes_read = read(drones[i].pipe_read, &pos, sizeof(Position));
            
            if (bytes_read == sizeof(Position)) {
                drones[i].x = pos.x;
                drones[i].y = pos.y;
                drones[i].z = pos.z;
                printf("Drone %d at position (%.2f, %.2f, %.2f) at time %.2f\n", 
                       i, pos.x, pos.y, pos.z, pos.time);
            }
        }
        
        // Check for collisions
        check_collisions();
        
        step++;
        usleep(100000);  // Sleep for 100ms between steps
    }
    
    printf("Simulation completed after %d steps\n", step);
}

void drone_process(Drone* drone, const char* script_file) {
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

void check_collisions() {
    for (int i = 0; i < drone_count; i++) {
        for (int j = i + 1; j < drone_count; j++) {
            // Calculate distance between drones
            double dx = drones[i].x - drones[j].x;
            double dy = drones[i].y - drones[j].y;
            double dz = drones[i].z - drones[j].z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < COLLISION_THRESHOLD) {
                printf("COLLISION ALERT: Drones %d and %d are too close (%.2f meters)!\n", 
                       i, j, distance);
                
                // In a real system, you might want to stop the simulation or take corrective action
                // For this example, we'll just report the collision
            }
        }
    }
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
    
    // In a real implementation, you might want to do more cleanup here
}
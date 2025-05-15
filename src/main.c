#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

#include "include/types.h"
#include "include/simulation.h"
#include "include/drone.h"
#include "include/report.h"
#include "include/utils.h"

// Define global variables
Drone drones[MAX_DRONES];
int drone_count = 0;
bool simulation_running = true;
int nlMax = 0;
bool collision_detected = false;
Collision collisions[MAX_COLLISIONS];
int collision_count = 0;
char figure_filename[256];
bool max_col = false;
int step = 0;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: %s <figure_file>\n", argv[0]);
        return 1;
    }

    strncpy(figure_filename, argv[1], sizeof(figure_filename) - 1);
    figure_filename[sizeof(figure_filename) - 1] = '\0';

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    initialize_simulation(argv[1]);
    start_simulation();
    generate_report();
    cleanup_simulation();
    
    return 0;
}
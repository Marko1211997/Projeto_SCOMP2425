#include <stdio.h>
#include <signal.h> 
#include <stdlib.h> 

#include "include/utils.h"
// Needs access to simulation_running global variable
#include "include/types.h" // Ensures simulation_running is declared extern

void signal_handler(int signum_param) { // Renamed param
    printf("Received signal %d, terminating...\n", signum_param); 
    simulation_running = false; 
}

int count_lines(const char* file_path) { // Renamed param
    FILE* file_handle = fopen(file_path, "r"); // Renamed
    if (!file_handle) {
        // perror("Error opening file for counting lines"); // Suppressed direct perror
        return -1; 
    }
    
    int line_count = 0; // Renamed
    char line_buffer[1024]; // Renamed
    while (fgets(line_buffer, sizeof(line_buffer), file_handle) != NULL) {
        line_count++;
    }
    fclose(file_handle);
    return line_count;
}
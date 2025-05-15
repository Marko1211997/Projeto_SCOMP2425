#ifndef SIMULATION_H
#define SIMULATION_H

#include "types.h"

void initialize_simulation(const char* figure_file);
void start_simulation();
void check_collisions(double time);
void cleanup_simulation();
bool check_active_drones();

#endif // SIMULATION_H
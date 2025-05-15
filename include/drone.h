#ifndef DRONE_H
#define DRONE_H

#include "types.h"

void drone_process(Drone* drone, const char* script_file);
void terminate_drone(int drone_id);
void terminate_drone_all();

#endif // DRONE_H
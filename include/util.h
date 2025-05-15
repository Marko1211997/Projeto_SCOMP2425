#ifndef UTILS_H
#define UTILS_H

#include "types.h" // For bool, and other types if necessary

void signal_handler(int signum);
int count_lines(const char* filename);

#endif // UTILS_H
#ifndef TYPES_H
#define TYPES_H

#include <stdbool.h>
#include <sys/types.h> // For pid_t

#define MAX_DRONES 100
#define MAX_STEPS 1000
#define MAX_COLLISIONS 4 // Número máximo de colisões
#define COLLISION_THRESHOLD 1.0  // Distância mínima entre drones (em metros)
#define REPORT_FILENAME "simulation_report.txt"

typedef struct {
    int id;
    double x, y, z;  // Coordenadas 3D
    pid_t pid;       // ID do processo do drone
    int pipe_read;   // Extremidade de leitura do pipe
    int pipe_write;  // Extremidade de escrita do pipe
    bool active;     // Flag para indicar se o drone ainda está ativo
} Drone;

typedef struct {
    double x, y, z; // Coordenadas da posição no espaço 3D
    double time;   // Tempo associado a esta posição
} Position;

typedef struct {
    int drone1_id;
    int drone2_id;
    double distance;
    double time; // Tempo em que a colisão ocorreu
    double x1, y1, z1; 
    double x2, y2, z2;  
} Collision;

// Global variables
extern Drone drones[MAX_DRONES];
extern int drone_count;
extern bool simulation_running;
extern int nlMax;
extern bool collision_detected;
extern Collision collisions[MAX_COLLISIONS];
extern int collision_count;
extern char figure_filename[256];
extern bool max_col;
extern int step;

#endif // TYPES_H
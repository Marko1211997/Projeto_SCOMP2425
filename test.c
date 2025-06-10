#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <errno.h>

//shared memory
#include <sys/mman.h>
#include <sys/stat.h> 
#include <fcntl.h> 
#include <sys/types.h>

//semaphores
#include <semaphore.h>

//threads
#include <pthread.h>

#define MAX_DRONES 100
#define MAX_STEPS 1000
#define MAX_COLLISIONS 10 // Número máximo de colisões
#define COLLISION_THRESHOLD 1.0 // Distância mínima entre drones (em metros)
#define REPORT_FILENAME "simulation_report.txt"

// semaphores
#define SHM_NAME "/drone_simulation_shm"
#define SEM_STEP_NAME "/step_semaphore"
#define SEM_BARRIER_NAME "/barrier_semaphore"

typedef struct
{
    int id;
    double x, y, z; // Coordenadas 3D
    pid_t pid; // ID do processo do drone
    double time; // Tempo associado a esta posição e step!
    int current_step; // Step atual do drone
    bool active; // Flag para indicar se o drone ainda está ativo
    bool completed; // Flag para indicar se o drone completou o seu script
    char script_file[256]; // Nome do ficheiro de script do drone

} Drone;

typedef struct
{
    int drone1_id;
    int drone2_id;
    double distance; // Distância entre os drones
    double time; // Tempo em que a colisão ocorreu
    double x1, y1, z1;
    double x2, y2, z2;
    bool processed; // Flag para indicar se a colisão já foi processada

} Collision;

//shared memory structure
typedef struct {
    Drone drones[MAX_DRONES];
    Collision collisions[MAX_COLLISIONS];
    int drone_count;
    int collision_count;
    int current_step;
    bool simulation_running;
    bool collision_detected;
    pthread_mutex_t mutex;
    pthread_cond_t step_cond;
    pthread_cond_t collision_cond;
    
} SharedMemory;

// Variáveis globais
SharedMemory *shared_mem = NULL;
sem_t *step_sem = NULL;
sem_t *barrier_sem = NULL;
//shared memory
int fd = -1;
void *ptr;
//threads
pthread_t collision_thread;
pthread_t report_thread;
//pthread_t monitor_thread;
bool threads_running = true;
//Drone drones[MAX_DRONES];
//int drone_count = 0;
//bool simulation_running = true;
int nlMax = 0;
//bool collision_detected = false;
//Collision collisions[MAX_COLLISIONS];
//int collision_count = 0;
char figure_filename[256];
bool max_col = false;
int step = 0;
volatile sig_atomic_t termination_requested = 0; 

// Declaração dos métodos

//thread functions
void* collision_detection_thread(void* arg);
void* report_generation_thread(void* arg);
//void* simulation_monitor_thread(void* arg);

//others
void initialize_simulation(const char *figure_file);
void start_simulation();
//void drone_process(Drone *drone, const char *script_file);
void drone_process(int drone_id, const char* script_file);
void check_collisions();
void cleanup_simulation();

void setup_signal_handling();
void handle_signal(int signum, siginfo_t *info, void *context);
//void signal_handler_termination(int signum);
//void signal_handler(int signum);
int count_lines(const char *filename);
void generate_report();

void terminate_drone();
void terminate_drone_all();

void alldronesReady();


//Create/open
//shm_open() //create a shared memory object
//ftruncate() //difine size
//mmap() //maps the shared memory
//int shm_open(const char *name, int oflag, mode_t mode);
//int ftruncate(int fd, off_t length);
//void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
void setup_shared_memory();
void setup_semaphores();


//clenup memory
//munmap() 
//close() 
//shm_unlink()
//int munmap(void *addr, size_t length);
//int close(int fd);
//int shm_unlink(const char *name);
void clenup_shared_memory();

void handle_signal(int signum, siginfo_t *info, void *context)
{
    if (signum == SIGUSR1) {

        printf("Drone process received termination signal, exiting (SIGUSR1)...\n");
        exit(EXIT_SUCCESS);

    } else {
        
        termination_requested = 1;
        if(shared_mem) {
            shared_mem->simulation_running = false;
        }
        threads_running = false;
        if(signum = SIGTERM){
            printf("Received signal (SIGTERM), terminating simulation...\n", signum);
        }else printf("Received signal (SIGINT), terminating simulation...\n", signum);

    }
}

void setup_signal_handling()
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa)); 
    sa.sa_flags = SA_SIGINFO | SA_RESTART;  
    sa.sa_sigaction = handle_signal;
    sigemptyset(&sa.sa_mask); 

    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("Failed to set SIGINT handler");
        exit(EXIT_FAILURE);
    }
    
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("Failed to set SIGTERM handler");
        exit(EXIT_FAILURE);
    }
    
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("Failed to set SIGUSR1 handler");
        exit(EXIT_FAILURE);
    }
    
    printf("Signal handling configured with sigaction\n");
}


int main(int argc, char *argv[])
{
    

    int option;
    do
    {
        printf("\n=== Figure Code Simulation System ===\n\n");
        printf("Choose an option:\n");
        printf("1 - Initiate Simulation\n");
        printf("2 - Exit\n");
        printf("Option: ");
        scanf("%d", &option);

        if (option != 1 && option != 2)
        {
            printf("Invalid option, please seleact try again.\n\n");
        }

    } while (option != 1 && option != 2);

    if (option == 1)
    {
        printf("Starting simulation...\n\n");

        if (argc != 2)
        {

            printf("Usage: %s <figure_file>\n", argv[0]);

            return 1;
        }

        // Armazena o nome do ficheiro da figura para o relatório

        strncpy(figure_filename, argv[1], sizeof(figure_filename) - 1);

        figure_filename[sizeof(figure_filename) - 1] = '\0';

        // Configura o manipulador de sinais para terminação graciosa

        setup_signal_handling();
        setup_shared_memory();
        setup_semaphores();
        initialize_simulation(argv[1]);
        start_simulation();
        generate_report();
        cleanup_simulation();
    }
    else if (option == 2)
    {
        printf("Exiting...\n");
    }

    return 0;
}

void setup_shared_memory()
{
    shm_unlink(SHM_NAME);
    sem_unlink(SEM_STEP_NAME);
    sem_unlink(SEM_BARRIER_NAME);

    // Create shared memory segment
    fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0644);
    if (fd == -1) {
        perror("shm_open failed");
        exit(EXIT_FAILURE);
    }

    if (ftruncate(fd, sizeof(SharedMemory)) == -1) {
        perror("ftruncate failed");
        exit(EXIT_FAILURE);
    }

    shared_mem = mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_mem == MAP_FAILED) {
        perror("mmap failed");
        exit(EXIT_FAILURE);
    }

    // Initialize shared memory
    memset(shared_mem, 0, sizeof(SharedMemory));
    shared_mem->simulation_running = true;
    shared_mem->collision_detected = false;
    shared_mem->current_step = 0;
    shared_mem->drone_count = 0;
    shared_mem->collision_count = 0;

    // Initialize mutex and condition variables for process sharing
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shared_mem->mutex, &mutex_attr);
    pthread_mutexattr_destroy(&mutex_attr);

    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&shared_mem->step_cond, &cond_attr);
    pthread_cond_init(&shared_mem->collision_cond, &cond_attr);
    pthread_condattr_destroy(&cond_attr);
}

void setup_semaphores()
{
    step_sem = sem_open(SEM_STEP_NAME, O_CREAT | O_EXCL, 0644, 0);
    barrier_sem = sem_open(SEM_BARRIER_NAME, O_CREAT | O_EXCL, 0644, 0);
    
    if (step_sem == SEM_FAILED || barrier_sem == SEM_FAILED) {
        perror("sem_open failed");
        exit(EXIT_FAILURE);
    }
}

// Função para inicializar a simulação, lendo a configuração de um ficheiro.

void initialize_simulation(const char *figure_file)
{
    FILE *file = fopen(figure_file, "r");
    if (!file)
    {
        perror("Error opening figure file!");
        exit(EXIT_FAILURE);
    }

    char line[256];
    char script_file[256];
    double x, y, z;

    // Lê as posições iniciais dos drones e os ficheiros de script do ficheiro de figura
    while (fgets(line, sizeof(line), file) && shared_mem->drone_count < MAX_DRONES)
    {
        if (sscanf(line, "%s %lf %lf %lf", script_file, &x, &y, &z) == 4)
        {
            shared_mem->drones[shared_mem->drone_count].id = shared_mem->drone_count;
            shared_mem->drones[shared_mem->drone_count].x = x;
            shared_mem->drones[shared_mem->drone_count].y = y;
            shared_mem->drones[shared_mem->drone_count].z = z;
            shared_mem->drones[shared_mem->drone_count].pid = 0;
            shared_mem->drones[shared_mem->drone_count].time = 0.0;
            shared_mem->drones[shared_mem->drone_count].current_step = 0;
            shared_mem->drones[shared_mem->drone_count].active = true;
            shared_mem->drones[shared_mem->drone_count].completed = false;
            strcpy(shared_mem->drones[shared_mem->drone_count].script_file, script_file);
            shared_mem->drone_count++;
        }

        int nL = count_lines(script_file);
        if (nL >= nlMax)
        {
            nlMax = nL;
        }
    }

    fclose(file);

    if (shared_mem->drone_count == 0){
        fprintf(stderr, "Error: No drones found in figure file!\n");
        exit(EXIT_FAILURE);
    }
    else{
        printf("Initialized %d drones for simulation\n", shared_mem->drone_count);
        
        printf("\nInitial Drone Positions - Time 0)\n");
        for (int i = 0; i < shared_mem->drone_count; i++) {
            printf("Drone %d: position (%.2f, %.2f, %.2f)\n", 
                i, shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z);
        }
        printf("\n");
        
    }
}

// Função para iniciar a simulação

void start_simulation()
{
    printf("Starting simulation with %d drones\n", shared_mem->drone_count);

    //Create Threads
    if (pthread_create(&collision_thread, NULL, collision_detection_thread, NULL) != 0) {
        perror("Failed to create collision detection thread");
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&report_thread, NULL, report_generation_thread, NULL) != 0) {
        perror("Failed to create report generation thread");
        exit(EXIT_FAILURE);
    }

    // Bifurca (ou cria) um processo para cada drone
    for (int i = 0; i < shared_mem->drone_count; i++){
        pid_t pid = fork();
        
        if (pid == -1)
        {
            perror("Fork failed!");
            exit(EXIT_FAILURE);
        }
        else if (pid == 0)
        {
            // Processo filho (drone)
            drone_process(i, shared_mem->drones[i].script_file);
            exit(EXIT_SUCCESS);
        }
        else
        {
            // Processo pai
            shared_mem->drones[i].pid = pid;
            printf("Started drone %d with PID %d using script %s\n", 
                   i, pid, shared_mem->drones[i].script_file);
        }
    }

    printf("\n");

    alldronesReady();

    // Loop de simulação principal

    shared_mem->current_step = 1;
    while (shared_mem->simulation_running && shared_mem->current_step < MAX_STEPS && 
        shared_mem->current_step < nlMax + 1 && !termination_requested &&
        shared_mem->collision_count < MAX_COLLISIONS){

        printf("\n-SIMULATION STEP %d-\n", shared_mem->current_step);

        int active_count = 0;
        for (int i = 0; i < shared_mem->drone_count; i++) {
            if (shared_mem->drones[i].active) {
                active_count++;
            }
        }
        if (active_count == 0) {
            printf("No active drones remaining\n");
            break;
        }
        printf("Signaling %d active drones to execute %d step\n", active_count, shared_mem->current_step);

        //pthread_mutex_lock(&shared_mem->mutex);
        //pthread_mutex_unlock(&shared_mem->mutex);

        for (int i = 0; i < active_count; i++) {
            sem_post(step_sem);
        }
        printf("Waiting for all drones to complete %d step\n", shared_mem->current_step);
        
        for (int i = 0; i < active_count; i++) {
            sem_wait(barrier_sem);
        }

        printf("All drones completed step %d\n", shared_mem->current_step);

        printf("\nAll Drone Positions at Step %d\n", shared_mem->current_step);
        for (int i = 0; i < shared_mem->drone_count; i++) {
            if (shared_mem->drones[i].active || shared_mem->drones[i].completed) {
                printf("Drone %d: position (%.2f, %.2f, %.2f) - %s\n", 
                       i, shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z,
                       shared_mem->drones[i].active ? "Active" : 
                       (shared_mem->drones[i].completed ? "Completed" : "Terminated"));
            }
        }
        printf("\n");

        // Verifica colisões

        check_collisions();

        if (shared_mem->collision_count >= MAX_COLLISIONS) {
            // Verifica se o número máximo de colisões foi atingido
            printf("\n*** COLLISION LIMIT EXCEEDED ***\n");
            printf("Detected %d collisions (limit: %d). Stopping simulation.\n", 
                   shared_mem->collision_count, MAX_COLLISIONS);
            max_col = true;
            shared_mem->simulation_running = false;
            terminate_drone_all();
            break;
        }

        printf("Step %d completed.\n", shared_mem->current_step);
        shared_mem->current_step++;


    }

    
    threads_running = false;
    shared_mem->simulation_running = false;

    pthread_cond_broadcast(&shared_mem->step_cond);
    pthread_cond_broadcast(&shared_mem->collision_cond);

    pthread_join(collision_thread, NULL);
    pthread_join(report_thread, NULL);

    printf("\nSimulation completed after %d steps\n", shared_mem->current_step - 1);
    printf("Total collisions: %d\n", shared_mem->collision_count);

}

// Esta função é executada por cada processo filho criado para simular um drone.

void drone_process(int drone_id, const char *script_file){
    setup_signal_handling();

    //open shared memory
    int drone_shm_fd = shm_open(SHM_NAME, O_RDWR, 0);
    if (drone_shm_fd == -1) {
        perror("Child: shm_open failed");
        exit(EXIT_FAILURE);
    }

    SharedMemory *drone_shared_mem = mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, drone_shm_fd, 0);
    if (drone_shared_mem == MAP_FAILED) {
        perror("Child: mmap failed");
        exit(EXIT_FAILURE);
    }

    //open semaphores
    sem_t *drone_step_sem = sem_open(SEM_STEP_NAME, 0);
    sem_t *drone_barrier_sem = sem_open(SEM_BARRIER_NAME, 0);

    if (drone_step_sem == SEM_FAILED || drone_barrier_sem == SEM_FAILED) {
        perror("Child: sem_open failed");
        exit(EXIT_FAILURE);
    }

    // Get initial position from shared memory
    double current_pos_x = drone_shared_mem->drones[drone_id].x;
    double current_pos_y = drone_shared_mem->drones[drone_id].y;
    double current_pos_z = drone_shared_mem->drones[drone_id].z;
    int script_line_number = 0; 

    printf("Drone %d started at position (%.2f, %.2f, %.2f)\n", 
           drone_id, current_pos_x, current_pos_y, current_pos_z);
    
    sem_post(drone_barrier_sem); // Signal that this drone is ready

    while (drone_shared_mem->simulation_running && !termination_requested) {
        
        if (sem_wait(drone_step_sem) == -1) {
            if (errno == EINTR) {
                if (termination_requested) break;
                continue;
            }
            perror("sem_wait failed");
            break;
        }

        if (!drone_shared_mem->simulation_running || termination_requested) {
            sem_post(drone_barrier_sem);
            break;
        }

        // Check if this drone is still active - if not, exit immediately
        if (!drone_shared_mem->drones[drone_id].active) {
            printf("Drone %d detected it was terminated due to collision, exiting process\n", drone_id);
            sem_post(drone_barrier_sem);
            break;  // Exit the loop and terminate this drone process
        }

        FILE *file = fopen(script_file, "r");
        if (!file){
            perror("Error opening drone script file!");
            exit(EXIT_FAILURE);
            break;
        }

        char line[256];
        char line_found = false;
        int current_line = 0;

        while (fgets(line, sizeof(line), file)) {
                if (current_line == script_line_number) {
                    line_found = true;
                    break;
                }
                current_line++;
        }
        fclose(file);
        if(line_found){
            double time, dx, dy, dz;
            if (sscanf(line, "%lf %lf %lf %lf", &time, &dx, &dy, &dz) == 4)
            {
                // Atualiza posição somando os deltas à posição atual
                current_pos_x += dx;
                current_pos_y += dy;
                current_pos_z += dz;
                script_line_number++;

                printf("Drone %d: Step %d - moved by (%.2f, %.2f, %.2f) to position (%.2f, %.2f, %.2f)\n", 
                        drone_id, drone_shared_mem->current_step, dx, dy, dz, current_pos_x, current_pos_y, current_pos_z);
                pthread_mutex_lock(&drone_shared_mem->mutex);

                if (drone_shared_mem->drones[drone_id].active){
                    drone_shared_mem->drones[drone_id].x = current_pos_x;
                    drone_shared_mem->drones[drone_id].y = current_pos_y;
                    drone_shared_mem->drones[drone_id].z = current_pos_z;
                    drone_shared_mem->drones[drone_id].time = time;
                    drone_shared_mem->drones[drone_id].current_step = script_line_number;
                } 
                pthread_mutex_unlock(&drone_shared_mem->mutex);
            }
        }else {
            printf("Drone %d reached end of script at simulation step %d\n", 
                drone_id, drone_shared_mem->current_step);
                
            pthread_mutex_lock(&drone_shared_mem->mutex);
            drone_shared_mem->drones[drone_id].completed = true;
            drone_shared_mem->drones[drone_id].active = false;
            pthread_mutex_unlock(&drone_shared_mem->mutex);
                
           printf("Drone %d completed its script at final position (%.2f, %.2f, %.2f)\n", 
                drone_id, current_pos_x, current_pos_y, current_pos_z);
                
            sem_post(drone_barrier_sem);
            break;
        }
        sem_post(drone_barrier_sem);
        
    }
    munmap(drone_shared_mem, sizeof(SharedMemory));
    close(drone_shm_fd);
    sem_close(drone_step_sem);
    sem_close(drone_barrier_sem);
        
    printf("Drone %d process exiting\n", drone_id); 
}

// Função para verificar e processar colisões entre drones num determinado instante de tempo da simulação

void check_collisions()
{
    printf("\nChecking for collisions\n");

    // Primeiro, identifica todas as colisões sem terminar nenhum drone

    bool will_terminate[MAX_DRONES] = {false};
    pthread_mutex_lock(&shared_mem->mutex);
    shared_mem->collision_detected = false;


    for (int i = 0; i < shared_mem->drone_count; i++){

        if (!shared_mem->drones[i].active){
            continue; // Avança drones inativos
        }

        for (int j = i + 1; j < shared_mem->drone_count; j++){
            // Calcula a distância entre drones
            if (!shared_mem->drones[j].active){
                continue; // Avança drones inativos
            }

            double dx = shared_mem->drones[i].x - shared_mem->drones[j].x;

            double dy = shared_mem->drones[i].y - shared_mem->drones[j].y;

            double dz = shared_mem->drones[i].z - shared_mem->drones[j].z;

            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < COLLISION_THRESHOLD)
            {
                printf("COLLISION ALERT: Drones %d and %d are too close (%.2f meters)!\n\n", i, j, distance);
                // Guarda as colisões
                if(shared_mem->collision_count < MAX_COLLISIONS){
                    shared_mem->collisions[shared_mem->collision_count].drone1_id = i;
                    shared_mem->collisions[shared_mem->collision_count].drone2_id = j;
                    shared_mem->collisions[shared_mem->collision_count].distance = distance;
                    shared_mem->collisions[shared_mem->collision_count].time = shared_mem->current_step;
                    shared_mem->collisions[shared_mem->collision_count].x1 = shared_mem->drones[i].x;
                    shared_mem->collisions[shared_mem->collision_count].y1 = shared_mem->drones[i].y;
                    shared_mem->collisions[shared_mem->collision_count].z1 = shared_mem->drones[i].z;
                    shared_mem->collisions[shared_mem->collision_count].x2 = shared_mem->drones[j].x;
                    shared_mem->collisions[shared_mem->collision_count].y2 = shared_mem->drones[j].y;
                    shared_mem->collisions[shared_mem->collision_count].z2 = shared_mem->drones[j].z;
                    shared_mem->collisions[shared_mem->collision_count].processed = false;
                    shared_mem->collision_count++;

                    will_terminate[i] = true;
                    will_terminate[j] = true;
                }

                shared_mem->collision_detected = true;
            }
        }
    }


    for (int i = 0; i < shared_mem->drone_count; i++)
    {

        if (will_terminate[i])
        {

            terminate_drone(i, SIGUSR1);
            pthread_cond_signal(&shared_mem->collision_cond);
            printf("Collision %d recorded. Drones %d TERMINATED. (Total collisions: %d/%d)\n", 
                       shared_mem->collision_count, i, shared_mem->collision_count, MAX_COLLISIONS);
        }
    }

    if (!shared_mem->collision_detected) {
        printf("No collisions detected at step %d\n", shared_mem->current_step);
    } else {
        printf("Total collisions so far: %d/%d\n", shared_mem->collision_count, MAX_COLLISIONS);
    }

    pthread_mutex_unlock(&shared_mem->mutex);
}

// Função para limpar os recursos da simulação

void cleanup_simulation()
{
    printf("Cleaning up simulation...\n");

    // Enviar sinal de término para todos os processos dos drones

    if (shared_mem) {
        for (int i = 0; i < shared_mem->drone_count; i++) {
            pid_t pid = shared_mem->drones[i].pid;
            if (pid > 0) {
                kill(pid, SIGTERM);
            }
        }
    }

    int status;
    pid_t pid;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        printf("Child process with PID %d terminated\n", pid);
    }

    clenup_shared_memory();

    printf("Simulation cleanup complete!\n");
}

void clenup_shared_memory(){  
    
    if (shared_mem && shared_mem != MAP_FAILED) {
        pthread_mutex_destroy(&shared_mem->mutex);
        pthread_cond_destroy(&shared_mem->step_cond);
        pthread_cond_destroy(&shared_mem->collision_cond);
        munmap(shared_mem, sizeof(SharedMemory));
    }

    if (fd >= 0) {
        close(fd);
        shm_unlink(SHM_NAME);
    }

    // Cleanup semaphores - following PDF pattern
    if (step_sem && step_sem != SEM_FAILED) {
        sem_close(step_sem);
        sem_unlink(SEM_STEP_NAME);
    }
    
    if (barrier_sem && barrier_sem != SEM_FAILED) {
        sem_close(barrier_sem);
        sem_unlink(SEM_BARRIER_NAME);
    }

}

// Função para contar o número de linhas num ficheiro

int count_lines(const char *filename)
{
    if (filename == NULL) return 0;
    FILE *file = fopen(filename, "r");
    if (!file)
    {
        perror("Error opening file");
        return -1;
    }
    int count = 0;

    // Declara um buffer de caracteres para armazenar temporariamente as linhas lidas do ficheiro.
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), file) != NULL){
        count++;
    }

    // Fecha o ficheiro após a leitura de todas as linhas.
    fclose(file);
    return count;
}

// Função para terminar um drone específico

void terminate_drone(int drone_id, int code)
{

    if (drone_id < 0 || drone_id >= shared_mem->drone_count)
    {

        return; // ID de drone inválido
    }

    if (!shared_mem->drones[drone_id].active)
    {

        return; // Drone já inativo
    }

    printf("Terminating drone %d \n", drone_id);

    // Enviar sinal de terminação para o processo do drone

    kill(shared_mem->drones[drone_id].pid, code);

    // Marcar o drone como inativo
    // Isso evita que o drone seja processado novamente na simulação.

    shared_mem->drones[drone_id].active = false;

    // Aguarda que o processo do drone termine

    waitpid(shared_mem->drones[drone_id].pid, NULL, 0);   

}

// Função para terminar todos os drones ativos

void terminate_drone_all()
{

    for (int i = 0; i < shared_mem->drone_count; i++)
    {

        if (shared_mem->drones[i].active)
        {

            terminate_drone(i, SIGTERM);
        }
    }
}

// Função para gerar o relatório da simulação

void generate_report()
{
    if (!shared_mem) return;
    FILE *report_file = fopen(REPORT_FILENAME, "w");
    if (!report_file){
        perror("Error creating report file!");
        return;
    }

    // Recupera a data e hora atual
    time_t current_time = time(NULL);
    char time_str[100];
    // Formata a data e hora para uma string legível
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&current_time));

    // Escreve o cabeçalho do relatório
    fprintf(report_file, "=======================================================\n");
    fprintf(report_file, "             DRONE FIGURE SIMULATION REPORT            \n");
    fprintf(report_file, "=======================================================\n\n");
    fprintf(report_file, "Generated: %s\n", time_str);
    fprintf(report_file, "Figure File: %s\n\n", figure_filename);
    // Escreve informações gerais da simulação
    fprintf(report_file, "-------------------------------------------------------\n");
    fprintf(report_file, "SUMMARY\n\n");
    fprintf(report_file, "Total Number of Drones: %d\n", shared_mem->drone_count);
    fprintf(report_file, "Total Steps: %d\n", shared_mem->current_step - 1);
    fprintf(report_file, "Total Collisions: %d\n", shared_mem->collision_count);
    fprintf(report_file, "Simulation Result: %s\n\n", (shared_mem->collision_count >= COLLISION_THRESHOLD) ? "FAILED (Collision limit exceeded)" :
     (shared_mem->collision_detected ? "FAILED (Collisions detected)" : "PASSED"));
    // Escreve informações sobre o estado dos drones
    fprintf(report_file, "-------------------------------------------------------\n");
    fprintf(report_file, "DRONE's STATUS\n\n");
    for (int i = 0; i < shared_mem->drone_count; i++){
        char script_file[256];
        fprintf(report_file, "Drone %d:\n", i);
        fprintf(report_file, "Script file: %s\n", shared_mem->drones[i].script_file);

        const char *status;
        if (shared_mem->drones[i].completed) {
            status = "Completed Successfully";
        } else if (!shared_mem->drones[i].active) {
            bool involved_in_collision = false;
            for (int j = 0; j < shared_mem->collision_count; j++) {
                if (shared_mem->collisions[j].drone1_id == i || 
                    shared_mem->collisions[j].drone2_id == i) {
                    involved_in_collision = true;
                    break;
                }
            }
            status = involved_in_collision ? "Terminated (Collision)" : "Terminated (Other)";
        } else {
            status = "Incomplete";
        }
        fprintf(report_file, "  Status: %s\n", status);
        fprintf(report_file, "  Final Position: (%.2f, %.2f, %.2f)\n", 
                shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z);
        fprintf(report_file, "  Steps Completed: %d\n\n", shared_mem->drones[i].current_step);
    }
        
    // Escreve informações sobre colisões
    if (shared_mem->collision_count > 0){
        fprintf(report_file, "-------------------------------------------------------\n");
        fprintf(report_file, "COLLISION(S) DETAILS\n\n");
        fprintf(report_file, "Total Number of Collisions: %d\n\n", shared_mem->collision_count);
        for (int i = 0; i < shared_mem->collision_count; i++){
            fprintf(report_file, "Collision %d:\n", i + 1);
            fprintf(report_file, "  Drones Involved: %d and %d\n",
                    shared_mem->collisions[i].drone1_id, shared_mem->collisions[i].drone2_id);
            fprintf(report_file, "  Time: %.2f seconds\n", shared_mem->collisions[i].time);
            fprintf(report_file, "  Distance between them: %.2f meters\n", shared_mem->collisions[i].distance);
            fprintf(report_file, "  Drone %d Position: (%.2f, %.2f, %.2f)\n",
                    shared_mem->collisions[i].drone1_id,
                    shared_mem->collisions[i].x1, shared_mem->collisions[i].y1, shared_mem->collisions[i].z1);
            fprintf(report_file, "  Drone %d Position: (%.2f, %.2f, %.2f)\n\n",
                    shared_mem->collisions[i].drone2_id,
                    shared_mem->collisions[i].x2, shared_mem->collisions[i].y2, shared_mem->collisions[i].z2);
        }
    }

    // Escreve recomendações
    fprintf(report_file, "-------------------------------------------------------\n");
    fprintf(report_file, "RECOMMENDATIONS\n\n");
    if (shared_mem->collision_count > 0){
        fprintf(report_file, "The figure is NOT safe to use.\nPlease modify the drone paths to avoid collisions.\n");
       
        // Sugestões para evitar colisões
        fprintf(report_file, "Consider adjusting the paths of the following drones:\n");
        for (int i = 0; i < shared_mem->collision_count; i++){
            fprintf(report_file, "- Drones %d and %d (collided at time %.2f)\n",
                   shared_mem->collisions[i].drone1_id, shared_mem->collisions[i].drone2_id, shared_mem->collisions[i].time);
        }
    }else{
        fprintf(report_file, "The figure is safe to use.\nAll drones completed their paths without collisions.\n");
    }

    fclose(report_file);
    printf("Simulation report generated: %s\n", REPORT_FILENAME);
}


void* collision_detection_thread(void* arg)
{
    printf("Collision detection thread started\n");

    while (threads_running && !termination_requested) {
        usleep(100000); // Sleep to avoid busy waiting
    }

    printf("Collision detection thread terminated\n");
    return NULL;
}

void* report_generation_thread(void* arg)
{
    printf("Report generation thread started\n");

    while (threads_running && !termination_requested) {
        pthread_mutex_lock(&shared_mem->mutex);

        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1;
        
        while (!shared_mem->collision_detected && threads_running && !termination_requested) {
            int result = pthread_cond_timedwait(&shared_mem->collision_cond, &shared_mem->mutex, &timeout);
            if (result == ETIMEDOUT) {
                if (termination_requested || !threads_running) break;
                clock_gettime(CLOCK_REALTIME, &timeout);
                timeout.tv_sec += 1;
            }
        }

        if (termination_requested || !threads_running) {
            pthread_mutex_unlock(&shared_mem->mutex);
            break;
        }

        // Process unprocessed collisions
        for (int i = 0; i < shared_mem->collision_count; i++) {
            if (!shared_mem->collisions[i].processed) {
                printf("Report: Processing collision between drones %d and %d at step %.0f\n",
                       shared_mem->collisions[i].drone1_id,
                       shared_mem->collisions[i].drone2_id,
                       shared_mem->collisions[i].time);
                shared_mem->collisions[i].processed = true;
            }
        }

        pthread_mutex_unlock(&shared_mem->mutex);
    }

    printf("Report generation thread terminated\n");
    return NULL;
}

void alldronesReady()
{
    printf("Waiting for all drones to be ready...\n");

    // Espera que todos os drones estejam prontos para iniciar a simulação
    for (int i = 0; i < shared_mem->drone_count; i++) {
        sem_wait(barrier_sem);
    }

    printf("All drones are ready to start the simulation!\n");
}

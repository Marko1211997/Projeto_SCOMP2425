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

// Bibliotecas para memória partilhada
#include <sys/mman.h>
#include <sys/stat.h> 
#include <fcntl.h> 
#include <sys/types.h>

// Bibliotecas para semáforos
#include <semaphore.h>

// Bibliotecas para threads
#include <pthread.h>

#define MAX_DRONES 100 // Número máximo de drones
#define MAX_STEPS 1000
#define MAX_COLLISIONS 10 // Número máximo de colisões
#define COLLISION_THRESHOLD 1.0 // Distância mínima entre drones (em metros)
#define REPORT_FILENAME "simulation_report.txt"

// Nomes para os objetos de sincronização (memória partilhada e semáforos)
#define SHM_NAME "/drone_simulation_shm"
#define SEM_BARRIER_NAME "/barrier_semaphore"
#define SEM_PHASE "/phase_semaphore"



// Estrutura para armazenar o estado de um único drone
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


// Estrutura para armazenar informações sobre uma colisão detetada
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

// Estrutura principal da memória partilhada que contém todo o estado da simulação
typedef struct {
   Drone drones[MAX_DRONES];             
    Collision collisions[MAX_COLLISIONS]; 
    int drone_count;                      
    int collision_count;                 
    int current_step;                     
    bool simulation_running;              // Flag que controla se a simulação principal está a decorrer
    bool collision_detected;              // Flag que indica se foi detetada uma colisão no passo atual
    pthread_mutex_t mutex;                // Mutex para proteger o acesso à memória partilhada
    pthread_cond_t step_cond;             // Variável de condição para sincronização de passos
    pthread_cond_t collision_cond;        // Variável de condição para sinalizar colisões
    pthread_cond_t ready;                 // Variável de condição para sincronizar threads

    int drones_completed_step;
    bool step_in_progress;
    
    bool threads_running;
    int nlMax;

    char figure_filename[256];

    volatile sig_atomic_t termination_requested; // Flag atómica para pedido de terminação (via sinal)

    bool collisions_checked;

} SharedMemory;

// Variáveis globais
SharedMemory *shared_mem = NULL;

int fd = -1;

// Semáforos
sem_t *barrier_sem = NULL; // Semáforo de barreira
sem_t *phase_sem = NULL; // Semaphore concrolo de fase
sem_t *drone_sem[MAX_DRONES]; // semaphore para cada drone

// Threads
pthread_t collision_thread; // Handle da thread de deteção de colisão
pthread_t report_thread;    // Handle da thread de geração de relatório


// Declaração dos métodos

// Funções das threads
void *collision_detection_thread(void *arg);
void *report_generation_thread(void *arg);


//Outras funções
void initialize_simulation(const char *figure_file);
void start_simulation();

void drone_process(int drone_id, const char* script_file);
void check_collisions();
void cleanup_simulation();

void setup_signal_handling();
void handle_signal(int signum, siginfo_t *info, void *context);

int count_lines(const char *filename);
void generate_report();

void terminate_drone();
void terminate_drone_all();

void alldronesReady();
void complete_all_active();
int count_active_drones();

void setup_shared_memory();
void setup_semaphores();

void clenup_shared_memory_semaphores();


// Função que trata os sinais recebidos (SIGINT, SIGTERM, SIGUSR1)
void handle_signal(int signum, siginfo_t *info, void *context)
{
    // Se o sinal for SIGUSR1, é para um processo drone terminar individualmente
    if (signum == SIGUSR1) {

        printf("Drone process received termination signal, exiting (SIGUSR1)...\n");
        exit(EXIT_SUCCESS);

    } else {
        // Define a flag de pedido de terminação
        shared_mem->termination_requested = 1;
        if(shared_mem) {
            shared_mem->simulation_running = false;
        }
        shared_mem->threads_running = false;
        if(signum = SIGTERM){
            printf("Received signal (SIGTERM), terminating simulation...\n", signum);
        }else printf("Received signal (SIGINT), terminating simulation...\n", signum);

    }
}

// Configura os handlers de sinais utilizando sigaction 
void setup_signal_handling()
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_flags = SA_SIGINFO | SA_RESTART; // Usa sa_sigaction e reinicia chamadas de sistema interrompidas
    sa.sa_sigaction = handle_signal;      // Define a função de tratamento
    sigemptyset(&sa.sa_mask);             // Não bloqueia outros sinais durante o tratamento


    // Regista o handler para SIGINT (Ctrl+C)
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("Failed to set SIGINT handler");
        exit(EXIT_FAILURE);
    }
    
    // Regista o handler para SIGTERM (sinal de terminação padrão)
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("Failed to set SIGTERM handler");
        exit(EXIT_FAILURE);
    }
    
    // Regista o handler para SIGUSR1 
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("Failed to set SIGUSR1 handler");
        exit(EXIT_FAILURE);
    }
    
    printf("Signal handling configured with sigaction\n");
}

// Função principal do programa
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

        // Verifica se o ficheiro da figura foi passado como argumento
        if (argc != 2)
        {

            printf("Usage: %s <figure_file>\n", argv[0]);

            return 1;
        }

        // Configura a memória partilhada, os semáforos e os handlers de sinal
        setup_shared_memory();
        setup_semaphores();
        setup_signal_handling();

        // Armazena o nome do ficheiro da figura para o relatório
        pthread_mutex_lock(&shared_mem->mutex);
        strncpy(shared_mem->figure_filename, argv[1], sizeof(shared_mem->figure_filename) - 1);

        shared_mem->figure_filename[sizeof(shared_mem->figure_filename) - 1] = '\0';
        pthread_mutex_unlock(&shared_mem->mutex);

        // Inicializa, executa e limpa a simulação
        initialize_simulation(argv[1]);
        start_simulation();
        cleanup_simulation();

    }
    else if (option == 2)
    {
        printf("Exiting...\n");
    }

    return 0;
}

// Configura e inicializa o segmento de memória partilhada
void setup_shared_memory()
{
    // Remove quaisquer instâncias antigas de memória partilhada ou semáforos com os mesmos nomes
    shm_unlink(SHM_NAME);
    sem_unlink(SEM_BARRIER_NAME);

    // Cria um novo segmento de memória partilhada
    fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0644);
    if (fd == -1) {
        perror("shm_open failed");
        exit(EXIT_FAILURE);
    }

    // Define o tamanho do segmento de memória partilhada
    if (ftruncate(fd, sizeof(SharedMemory)) == -1) {
        perror("ftruncate failed");
        exit(EXIT_FAILURE);
    }

    // Mapeia o segmento de memória para o espaço de endereçamento do processo
    shared_mem = mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_mem == MAP_FAILED) {
        perror("mmap failed");
        exit(EXIT_FAILURE);
    }

    // Inicializa a memória partilhada com valores padrão
    memset(shared_mem, 0, sizeof(SharedMemory));
    shared_mem->simulation_running = true;
    shared_mem->collision_detected = false;
    shared_mem->current_step = 0;
    shared_mem->drone_count = 0;
    shared_mem->collision_count = 0;
    shared_mem->drones_completed_step = 0;
    shared_mem->step_in_progress = false;
    shared_mem->threads_running = true;
    shared_mem->nlMax = 0;
    shared_mem->collisions_checked = false;


    // Inicializa o mutex e as variáveis de condição para serem partilháveis entre processos
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
    pthread_cond_init(&shared_mem->ready, &cond_attr);
    pthread_condattr_destroy(&cond_attr);
}

// Configura e inicializa os semáforos nomeados
void setup_semaphores()
{

    // Cria o semáforo de barreira, inicializado a 0
    barrier_sem = sem_open(SEM_BARRIER_NAME, O_CREAT | O_EXCL, 0644, 0);
    // Cria o semáforo de fase, inicializado a 1
    phase_sem = sem_open(SEM_PHASE, O_CREAT | O_EXCL, 0644, 1);

    // Cria um semáforo individual para cada drone, inicializados a 0
    char sem_drone[32];
    for (int i= 0; i < MAX_DRONES; i++){
        sprintf(sem_drone, "/drone_sem_%d", i);
    
        drone_sem[i]= sem_open(sem_drone, O_CREAT | O_EXCL, 0644, 0);
        if (drone_sem[i] == SEM_FAILED){
            perror("sem_open failed for a drone semaphore");
            exit(EXIT_FAILURE);
        }
    }

    if (barrier_sem == SEM_FAILED) {
        perror("sem_open failed");
        exit(EXIT_FAILURE);
    }
}

// Função para inicializar a simulação, lendo a configuração de um ficheiro.
void initialize_simulation(const char *figure_file)
{
    pthread_mutex_lock(&shared_mem->mutex);

    FILE *file = fopen(figure_file, "r");
    if (!file)
    {
        perror("Error opening figure file!");
        cleanup_simulation();

        exit(EXIT_FAILURE);
    }

    char line[256];
    char script_file[256];
    double x, y, z;

    // Lê as posições iniciais dos drones e os ficheiros de script do ficheiro de figura
    while (fgets(line, sizeof(line), file) && shared_mem->drone_count < MAX_DRONES)
    {
        // Interpreta cada linha para obter o nome do script e as coordenadas iniciais
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

        // Conta as linhas no ficheiro de script para determinar a duração da simulação
        int nL = count_lines(script_file);
        if (nL >= shared_mem->nlMax)
        {
            shared_mem->nlMax = nL;
        }
    }

    fclose(file);

    if (shared_mem->drone_count == 0){
        fprintf(stderr, "Error: No drones found in figure file!\n");
        exit(EXIT_FAILURE);
    }
        
    pthread_mutex_unlock(&shared_mem->mutex);

}

// Função para iniciar e gerir o loop principal da simulação
void start_simulation()
{
    printf("Starting simulation with %d drones\n", shared_mem->drone_count);

    // Cria as threads de deteção de colisão e de geração de relatório
    if (pthread_create(&collision_thread, NULL, collision_detection_thread, NULL) != 0) {
        perror("Failed to create collision detection thread");
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&report_thread, NULL, report_generation_thread, NULL) != 0) {
        perror("Failed to create report generation thread");
        exit(EXIT_FAILURE);
    }

    // Bifurca (cria) um processo filho para cada drone
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
    pthread_mutex_lock(&shared_mem->mutex);
    shared_mem->current_step = 1;
    pthread_mutex_unlock(&shared_mem->mutex);

    // O loop continua enquanto a simulação estiver ativa, dentro dos limites de passos e colisões
    while (shared_mem->simulation_running && shared_mem->current_step < MAX_STEPS && 
        shared_mem->current_step < shared_mem->nlMax + 1 && !shared_mem->termination_requested &&
        shared_mem->collision_count < MAX_COLLISIONS){

        printf("\n-SIMULATION STEP %d-\n", shared_mem->current_step);

        int active_count = count_active_drones();
        if (active_count == 0) {
            printf("No active drones.\n");
            pthread_mutex_lock(&shared_mem->mutex);
            shared_mem->simulation_running = false;
            pthread_mutex_unlock(&shared_mem->mutex);
            break;
        }
        //sem_wait(phase_sem);
        // Reseta os contadores para o novo passo
        pthread_mutex_lock(&shared_mem->mutex);
        shared_mem->drones_completed_step = 0;
        shared_mem->collisions_checked = false;
        pthread_mutex_unlock(&shared_mem->mutex);
        printf("Signaling %d active drones to execute %d step\n", active_count, shared_mem->current_step);

        // Acorda cada drone ativo para executar o seu próximo movimento
        for (int i = 0; i < shared_mem->drone_count; i++) {
            if (shared_mem->drones[i].active){
                sem_post(drone_sem[i]);
            }
        }

        printf("Waiting for all drones to complete %d step\n", shared_mem->current_step);
        
        // Espera na barreira até que todos os drones ativos tenham completado o passo
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

        //check_collisions();

        // Sinaliza a thread de deteção de colisão para começar a verificar
        pthread_mutex_lock(&shared_mem->mutex);
        shared_mem->step_in_progress = true;
        pthread_cond_signal(&shared_mem->ready);
        while (!shared_mem->collisions_checked && shared_mem->simulation_running) {
            pthread_cond_wait(&shared_mem->ready, &shared_mem->mutex);
        }
        pthread_mutex_unlock(&shared_mem->mutex);
        if (shared_mem->collision_count >= MAX_COLLISIONS) {
            // Verifica se o número máximo de colisões foi atingido
            printf("\n*** COLLISION LIMIT EXCEEDED ***\n");
            printf("Detected %d collisions (limit: %d). Stopping simulation.\n", 
                   shared_mem->collision_count, MAX_COLLISIONS);
            pthread_mutex_lock(&shared_mem->mutex);
            shared_mem->simulation_running = false;
            pthread_mutex_unlock(&shared_mem->mutex);
            terminate_drone_all();

        }

        printf("Step %d completed.\n", shared_mem->current_step);
        // Avança para o próximo passo
        pthread_mutex_lock(&shared_mem->mutex);
        shared_mem->current_step++;
        shared_mem->step_in_progress = false;
        pthread_mutex_unlock(&shared_mem->mutex);

    }

    // Se a simulação terminou sem exceder o limite de colisões, marca os drones ativos como completos
    if(shared_mem->collision_count < MAX_COLLISIONS){
        complete_all_active();
    }
    
    // Sinaliza o fim da simulação para as threads
    shared_mem->threads_running = false;
    shared_mem->simulation_running = false;

    // Acorda quaisquer threads que possam estar a dormir para que possam terminar
    pthread_mutex_lock(&shared_mem->mutex);
    pthread_cond_broadcast(&shared_mem->collision_cond);
    pthread_cond_broadcast(&shared_mem->step_cond);
    pthread_cond_broadcast(&shared_mem->ready);
    pthread_mutex_unlock(&shared_mem->mutex);

    // Espera que as threads terminem a sua execução
    pthread_join(collision_thread, NULL);
    pthread_join(report_thread, NULL);

    printf("\nSimulation completed after %d steps\n", shared_mem->current_step - 1);
    printf("Total collisions: %d\n", shared_mem->collision_count);
}

// Esta função é executada por cada processo filho criado para simular um drone.

void drone_process(int drone_id, const char *script_file){
    // Cada processo drone configura o seu próprio handler de sinais
    setup_signal_handling();

    // Abre a memória partilhada existente
    int drone_shm_fd = shm_open(SHM_NAME, O_RDWR, 0);
    if (drone_shm_fd == -1) {
        perror("Child: shm_open failed");
        exit(EXIT_FAILURE);
    }

    // Mapeia a memória partilhada para o seu espaço de endereçamento
    SharedMemory *drone_shared_mem = mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, drone_shm_fd, 0);
    if (drone_shared_mem == MAP_FAILED) {
        perror("Child: mmap failed");
        exit(EXIT_FAILURE);
    }

    // Abre o semáforo de barreira
    sem_t *drone_barrier_sem = sem_open(SEM_BARRIER_NAME, 0);

    // Obtém a sua posição inicial da memória partilhada
    pthread_mutex_lock(&drone_shared_mem->mutex);
    double current_pos_x = drone_shared_mem->drones[drone_id].x;
    double current_pos_y = drone_shared_mem->drones[drone_id].y;
    double current_pos_z = drone_shared_mem->drones[drone_id].z;
    pthread_mutex_unlock(&drone_shared_mem->mutex);
    int script_line_number = 0; 

    printf("Drone %d ready to start at position (%.2f, %.2f, %.2f)\n", 
           drone_id, current_pos_x, current_pos_y, current_pos_z);
    
    // Sinaliza que este drone está pronto, incrementando o semáforo de barreira   
    sem_post(drone_barrier_sem); 

    // Loop principal do drone
    while (drone_shared_mem->simulation_running && !shared_mem->termination_requested) {
        
        // Espera pelo seu semáforo individual, que será libertado pelo processo principal no início de cada passo
        if (sem_wait(drone_sem[drone_id]) == -1) {
            if (errno == EINTR) {
                if (shared_mem->termination_requested) break;
                continue;
            }
            perror("sem_wait failed");
            break;
        }

        // Verifica novamente as condições de terminação após ser acordado
        if (!drone_shared_mem->simulation_running || shared_mem->termination_requested) {
            sem_post(drone_barrier_sem);
            break;
        }

        // Verifica se este drone foi desativado (devido a uma colisão)
        if (!drone_shared_mem->drones[drone_id].active) {
            printf("Drone %d detected it was terminated due to collision, exiting process\n", drone_id);
            sem_post(drone_barrier_sem); // Liberta a barreira
            break;  // Sai do loop e termina o processo
        }

        // Abre o seu ficheiro de script para ler o próximo movimento
        FILE *file = fopen(script_file, "r");
        if (!file){
            perror("Error opening drone script file!");
            exit(EXIT_FAILURE);
            break;
        }

        char line[256];
        char line_found = false;
        int current_line = 0;

        // Lê o ficheiro linha por linha até encontrar a linha correspondente ao passo atual
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

                // Atualiza a sua posição na memória partilhada
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
        }

        // Sinaliza na barreira que completou o seu passo
        sem_post(drone_barrier_sem);
        
    }
    // Limpa os recursos antes de terminar
    munmap(drone_shared_mem, sizeof(SharedMemory));
    close(drone_shm_fd);
    sem_close(drone_barrier_sem);
        
    printf("Drone %d process exiting\n", drone_id); 
}

// Função para verificar e processar colisões entre drones num determinado instante de tempo da simulação
void check_collisions()
{
    printf("\nChecking for collisions\n");

    // Primeiro, identifica todas as colisões sem terminar nenhum drone
    bool will_terminate[MAX_DRONES] = {false};
    //pthread_mutex_lock(&shared_mem->mutex);
    shared_mem->collision_detected = false;

    // Itera por todos os pares de drones ativos
    for (int i = 0; i < shared_mem->drone_count; i++){

        if (!shared_mem->drones[i].active){
            continue; // Avança drones inativos
        }

        for (int j = i + 1; j < shared_mem->drone_count; j++){
            // Calcula a distância entre drones
            if (!shared_mem->drones[j].active){
                continue; // Avança drones inativos
            }

            // Calcula a distância euclidiana entre os dois drones
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

    // Termina todos os drones que foram marcados para terminação
    for (int i = 0; i < shared_mem->drone_count; i++)
    {

        if (will_terminate[i])
        {

            terminate_drone(i, SIGUSR1);
            //printf("Collision %d recorded. Drones %d TERMINATED. (Total collisions: %d/%d)\n", 
            //           shared_mem->collision_count, i, shared_mem->collision_count, MAX_COLLISIONS);
        }
    }

    if (!shared_mem->collision_detected) {
        printf("No collisions detected at step %d\n", shared_mem->current_step);
    } else {
        pthread_cond_signal(&shared_mem->collision_cond);

        // Sinaliza a thread de relatório que houve uma nova colisão
        printf("Total collisions so far: %d/%d\n", shared_mem->collision_count, MAX_COLLISIONS);
    }
    //pthread_mutex_unlock(&shared_mem->mutex);
}

// Função para limpar os recursos da simulação
void cleanup_simulation()
{
    printf("Cleaning up simulation...\n");

    // Envia sinal de término para todos os processos filho (drones) ainda ativos
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

    // Espera que todos os processos filho terminem
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        printf("Child process with PID %d terminated\n", pid);
    }

    clenup_shared_memory_semaphores();

    printf("Simulation cleanup complete!\n");
}

// Função dedicada a libertar a memória partilhada e os semáforos
void clenup_shared_memory_semaphores(){  
    
    if (shared_mem && shared_mem != MAP_FAILED) {
        pthread_mutex_destroy(&shared_mem->mutex);
        pthread_cond_destroy(&shared_mem->step_cond);
        pthread_cond_destroy(&shared_mem->collision_cond);
        munmap(shared_mem, sizeof(SharedMemory));
    }

    // Fecha e remove o ficheiro de memória partilhada
    if (fd >= 0) {
        close(fd);
        shm_unlink(SHM_NAME);
    }

    // Limpa os semáforos nomeados    
    if (barrier_sem && barrier_sem != SEM_FAILED) {
        sem_close(barrier_sem);
        sem_unlink(SEM_BARRIER_NAME);
    }
    if (phase_sem && phase_sem != SEM_FAILED) {
        sem_close(phase_sem);
        sem_unlink(SEM_PHASE);
    }

    // Limpa os semáforos individuais dos drones
    char drone_semaphore[32];
    for (int i = 0; i < MAX_DRONES; i++) {
        if (drone_sem[i] && drone_sem[i] != SEM_FAILED) {
            sem_close(drone_sem[i]);
            sprintf(drone_semaphore,"/drone_sem_%d",i);
            sem_unlink(drone_semaphore);
        }
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
    //tive que comentar por confilto com semaphoros, quando terminava o resto dos drones por excesso de colisoes, ponto negativo ficamos com processos filhos zombie //até a data sem solução
    //waitpid(shared_mem->drones[drone_id].pid, NULL, 0);  
    
}

// Função para terminar todos os drones ativos
void terminate_drone_all()
{

    for (int i = 0; i < shared_mem->drone_count; i++)
    {

        if (shared_mem->drones[i].active)
        {
            pthread_mutex_lock(&shared_mem->mutex);
            terminate_drone(i, SIGTERM);
            pthread_cond_signal(&shared_mem->collision_cond);~
            pthread_mutex_unlock(&shared_mem->mutex);


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
    fprintf(report_file, "Figure File: %s\n\n", shared_mem->figure_filename);
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
            status = involved_in_collision ? "Terminated (Collision)" : "Terminated (Incompleted)";
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

// Função executada pela thread de deteção de colisões
void* collision_detection_thread(void* arg)
{
    printf("Collision detection thread started\n");

    while (shared_mem->threads_running && !shared_mem->termination_requested) {
        pthread_mutex_lock(&shared_mem->mutex);

        // Espera até que um passo de simulação esteja em progresso
        while (!shared_mem->step_in_progress && shared_mem->threads_running && !shared_mem->termination_requested) {
            pthread_cond_wait(&shared_mem->ready, &shared_mem->mutex);
        }

        // Sai do loop se a simulação terminou
        if (!shared_mem->threads_running || shared_mem->termination_requested ) {
            pthread_mutex_unlock(&shared_mem->mutex);
            break;
        }

        // Verifica colisões
        if(shared_mem->step_in_progress && !shared_mem->collisions_checked){
            check_collisions();
            // Notifica a thread de geração de relatórios que as colisões foram verificadas
            shared_mem->collisions_checked = true;
            pthread_cond_signal(&shared_mem->ready);
        }
        pthread_mutex_unlock(&shared_mem->mutex);
    }

    

    printf("Collision detection thread terminated\n");
    return NULL;
}

// Função para a thread de geração de relatórios
void* report_generation_thread(void* arg)
{
    printf("Report generation thread started\n");

    while (shared_mem->threads_running && !shared_mem->termination_requested) {
        pthread_mutex_lock(&shared_mem->mutex);

        // Processa collisiões não processadas
        for (int i = 0; i < shared_mem->collision_count; i++) {
            if (!shared_mem->collisions[i].processed) {
               // printf("Report: Processing collision between drones %d and %d at step %.0f\n",
               //        shared_mem->collisions[i].drone1_id,
               //        shared_mem->collisions[i].drone2_id,
               //        shared_mem->collisions[i].time);
                shared_mem->collisions[i].processed = true;
            }
        }

        pthread_mutex_unlock(&shared_mem->mutex);
    }
    
    // Gera o relatório final uma vez que a simulação tenha terminado
    generate_report();

    printf("Report generation thread terminated\n");
    return NULL;
}

// Função de sincronização que espera que todos os drones estejam prontos
void alldronesReady()
{
    printf("Waiting for all drones to be ready...\n");

    // Espera que todos os drones estejam prontos para iniciar a simulação
    for (int i = 0; i < shared_mem->drone_count; i++) {
        sem_wait(barrier_sem);
    }

    printf("All drones are ready to start the simulation!\n");
}

// Marca todos os drones que ainda estão ativos como "completos" no final da simulação
void complete_all_active(){
    // Completa todos os drones ativos
    for (int i = 0; i < shared_mem->drone_count; i++) {
        if (shared_mem->drones[i].active) {
            shared_mem->drones[i].completed = true;
        }
    }

    // Acorda qualquer thread que possa estar à espera
    pthread_cond_broadcast(&shared_mem->step_cond);
}

// Função utilitária para contar o número de drones atualmente ativos
int count_active_drones()
{
    int count=0;
    for (int i = 0; i < shared_mem->drone_count; i++) {
            if (shared_mem->drones[i].active) {
                count++;
            }
    }
    return count;

}
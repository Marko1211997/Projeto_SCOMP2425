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
#define MAX_COLLISIONS 10 // Número máximo de colisões
#define COLLISION_THRESHOLD 1.0 // Distância mínima entre drones (em metros)
#define REPORT_FILENAME "simulation_report.txt"

typedef struct
{

    int id;
    double x, y, z; // Coordenadas 3D
    pid_t pid; // ID do processo do drone
    int pipe_read; // Extremidade de leitura do pipe
    int pipe_write; // Extremidade de escrita do pipe
    bool active; // Flag para indicar se o drone ainda está ativo
    char script_file[256]; // Nome do ficheiro de script do drone

} Drone;

typedef struct
{

    double x, y, z; // Coordenadas da posição no espaço 3D
    double time; // Tempo associado a esta posição

} Position;

typedef struct
{

    int drone1_id;
    int drone2_id;
    double distance; // Distância entre os drones
    double time; // Tempo em que a colisão ocorreu
    double x1, y1, z1;
    double x2, y2, z2;

} Collision;

// Variáveis globais

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

// Declaração dos métodos

void initialize_simulation(const char *figure_file);
void start_simulation();
void drone_process(Drone *drone, const char *script_file);
void check_collisions();
void cleanup_simulation();
void signal_handler_termination(int signum);
void signal_handler(int signum);
int count_lines(const char *filename);
void generate_report();
void terminate_drone();
bool check_active_drones();
void terminate_drone_all();

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

        signal(SIGUSR1, signal_handler);

        signal(SIGTERM, signal_handler_termination);

        signal(SIGINT, signal_handler);

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
    while (fgets(line, sizeof(line), file) && drone_count < MAX_DRONES)
    {
        if (sscanf(line, "%s %lf %lf %lf", script_file, &x, &y, &z) == 4)
        {
            drones[drone_count].id = drone_count;
            drones[drone_count].x = x;
            drones[drone_count].y = y;
            drones[drone_count].z = z;
            drones[drone_count].active = true;
            
            // Armazena o nome do ficheiro de script
            strncpy(drones[drone_count].script_file, script_file, sizeof(drones[drone_count].script_file) - 1);
            drones[drone_count].script_file[sizeof(drones[drone_count].script_file) - 1] = '\0';

            // Cria pipe para comunicação
            int pipe_fd[2];
            if (pipe(pipe_fd) == -1)
            {
                perror("Pipe creation failed");
                exit(EXIT_FAILURE);
            }

            drones[drone_count].pipe_read = pipe_fd[0];
            drones[drone_count].pipe_write = pipe_fd[1];

            drone_count++;
        }

        int nL = count_lines(script_file);
        if (nL >= nlMax)
        {
            nlMax = nL;
        }
    }

    fclose(file);

    if (drone_count == 0)
    {

        fprintf(stderr, "Error: No drones found in figure file!\n");

        exit(EXIT_FAILURE);
    }
    else
        printf("Initialized %d drones for simulation\n", drone_count);
}

// Função para iniciar a simulação

void start_simulation()
{
    printf("Starting simulation with %d drones\n", drone_count);

    // Bifurca (ou cria) um processo para cada drone
    for (int i = 0; i < drone_count; i++)
    {
        pid_t pid = fork();

        if (pid == -1)
        {
            perror("Fork failed!");
            exit(EXIT_FAILURE);
        }
        else if (pid == 0)
        {
            // Processo filho (drone)
        
            drone_process(&drones[i], drones[i].script_file);
            exit(EXIT_SUCCESS);
        }
        else
        {
            // Processo pai
            drones[i].pid = pid;
            printf("Started drone %d with PID %d using script %s\n", 
                   i, pid, drones[i].script_file);
        }
    }

    printf("\n");

    // Loop de simulação principal

    while (simulation_running && step < MAX_STEPS && step < nlMax + 1)
    {

        // Lê posições de todos os drones

        double time = 0.0;

        bool any_active = false;

        // Itera por todos os drones que foram inicializados

        for (int i = 0; i < drone_count; i++)
        {

            if (!drones[i].active)
            {

                continue; // Avança drones inativos
            }

            any_active = true;

            Position pos;

            // Tenta ler os dados de posição do pipe de leitura do drone atual

            ssize_t bytes_read = read(drones[i].pipe_read, &pos, sizeof(Position));

            if (bytes_read == sizeof(Position))
            {

                // Se a leitura foi bem-sucedida, atualiza as coordenadas x, y, z do drone

                drones[i].x = pos.x;

                drones[i].y = pos.y;

                drones[i].z = pos.z;

                printf("Drone %d at position (%.2f, %.2f, %.2f) at time %.2f\n",

                       i, pos.x, pos.y, pos.z, pos.time);

                time = pos.time;
            }
        }

        if (!any_active)
        {

            printf("All drones have completed or been terminated\n");

            break;
        }

        printf("\n");

        // Verifica colisões

        check_collisions(time);

        step++;
    }

    // Se chegar ao fim de todos os scripts sem colisões, marcar todos os drones como concluídos

    if (!collision_detected && step >= nlMax)
    {

        for (int i = 0; i < drone_count; i++)
        {

            drones[i].active = true;
        }
    }

    printf("Simulation completed after %d steps\n", step - 1);
}

// Função que define o comportamento de cada processo individual de drone.

// Esta função é executada por cada processo filho criado para simular um drone.

void drone_process(Drone *drone, const char *script_file)
{
    // Configura o manipulador de sinais para terminação
    signal(SIGUSR1, signal_handler);

    // Fecha a extremidade de leitura do pipe no processo do drone
    close(drone->pipe_read);

    FILE *file = fopen(script_file, "r");
    if (!file)
    {
        perror("Error opening drone script file!");
        exit(EXIT_FAILURE);
    }

    char line[256];
    double time, delta_x, delta_y, delta_z;

    // Posição inicial (mantém a posição inicial do drone)
    Position current_pos = {drone->x, drone->y, drone->z, 0.0};

    // Envia posição inicial para o processo principal
    write(drone->pipe_write, &current_pos, sizeof(Position));

    // Executa script de movimento
    while (fgets(line, sizeof(line), file) && simulation_running)
    {
        if (sscanf(line, "%lf %lf %lf %lf", &time, &delta_x, &delta_y, &delta_z) == 4)
        {
            // Atualiza posição somando os deltas à posição atual
            current_pos.x += delta_x;
            current_pos.y += delta_y;
            current_pos.z += delta_z;
            current_pos.time = time;

            // Manda posição para o processo principal
            write(drone->pipe_write, &current_pos, sizeof(Position));

        }
    }

    fclose(file);
    close(drone->pipe_write);
}

// Função para verificar e processar colisões entre drones num determinado instante de tempo da simulação

void check_collisions(double time)
{

    // Primeiro, identifica todas as colisões sem terminar nenhum drone

    bool will_terminate[MAX_DRONES] = {false};

    for (int i = 0; i < drone_count; i++)
    {

        if (!drones[i].active)
        {

            continue; // Avança drones inativos
        }

        for (int j = i + 1; j < drone_count; j++)
        {

            // Calcula a distância entre drones

            if (!drones[i].active || !drones[j].active)
            {

                continue; // Avança drones inativos
            }

            double dx = drones[i].x - drones[j].x;

            double dy = drones[i].y - drones[j].y;

            double dz = drones[i].z - drones[j].z;

            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < COLLISION_THRESHOLD)
            {

                printf("COLLISION ALERT: Drones %d and %d are too close (%.2f meters)!\n\n",

                       i, j, distance);

                // Guarda as colisões

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

                collision_detected = true;

                will_terminate[i] = true;

                will_terminate[j] = true;
            }
        }
    }

    for (int i = 0; i < drone_count; i++)
    {

        if (will_terminate[i])
        {

            terminate_drone(i, SIGUSR1);
        }
    }

    if (collision_detected)
    {

        printf("\n");
    }

    // Verifica se o número máximo de colisões foi atingido

    if (collision_count >= MAX_COLLISIONS)
    {

        printf("Maximum number of collisions reached, stopping simulation!\n");

        max_col = true;

        simulation_running = false;

        terminate_drone_all();
    }
}

// Função para verificar se ainda existem drones ativos na simulação

bool check_active_drones()
{

    for (int i = 0; i < drone_count; i++)
    {

        if (drones[i].active)
        {

            return true; // Pelo menos um drone ainda está ativo
        }
    }

    return false; // Nenhum drone ativo restante
}

// Função para limpar os recursos da simulação

void cleanup_simulation()
{

    // Enviar sinal de término para todos os processos dos drones

    for (int i = 0; i < drone_count; i++)
    {

        kill(drones[i].pid, SIGTERM);

        close(drones[i].pipe_read);

        close(drones[i].pipe_write);
    }

    // Aguardar que todos os processos filhos terminem

    for (int i = 0; i < drone_count; i++)
    {

        waitpid(drones[i].pid, NULL, 0);
    }

    printf("Simulation cleanup complete!\n");
}

// Função manipuladora de sinais (signal handler)

void signal_handler(int signum)
{

    // Imprime uma mensagem indicando que um sinal foi recebido e qual o número do sinal.

    printf("Received signal %d (SIGUSR1), terminating...\n", signum);

    sigprocmask();

    simulation_running = false;
}

// Função manipuladora de sinais (signal handler for termination)

void signal_handler_termination(int signum)
{

    // Imprime uma mensagem indicando que um sinal foi recebido e qual o número do sinal.

    printf("Received signal %d (SIGTERM), terminating...\n", signum);

    sigprocmask();

    simulation_running = false;
}

// Função para contar o número de linhas num ficheiro

int count_lines(const char *filename)
{

    FILE *file = fopen(filename, "r");

    if (!file)
    {

        perror("Error opening file");

        return -1;
    }

    int count = 0;

    // Declara um buffer de caracteres para armazenar temporariamente as linhas lidas do ficheiro.

    char buffer[1024];

    while (fgets(buffer, sizeof(buffer), file) != NULL)
    {

        count++;
    }

    // Fecha o ficheiro após a leitura de todas as linhas.

    fclose(file);

    return count;
}

// Função para terminar um drone específico

void terminate_drone(int drone_id, int code)
{

    if (drone_id < 0 || drone_id >= drone_count)
    {

        return; // ID de drone inválido
    }

    if (!drones[drone_id].active)
    {

        return; // Drone já inativo
    }

    printf("Terminating drone %d \n", drone_id);

    // Enviar sinal de terminação para o processo do drone

    kill(drones[drone_id].pid, code);

    // Marcar o drone como inativo
    // Isso evita que o drone seja processado novamente na simulação.

    drones[drone_id].active = false;

    // Aguarda que o processo do drone termine

    waitpid(drones[drone_id].pid, NULL, 0);

    // Fecha os pipes de leitura e escrita

    close(drones[drone_id].pipe_read);

    close(drones[drone_id].pipe_write);
}

// Função para terminar todos os drones ativos

void terminate_drone_all()
{

    for (int i = 0; i < drone_count; i++)
    {

        if (drones[i].active)
        {

            terminate_drone(i, SIGTERM);
        }
    }
}

// Função para gerar o relatório da simulação

void generate_report()
{

    FILE *report_file = fopen(REPORT_FILENAME, "w");

    if (!report_file)
    {

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

    fprintf(report_file, "Total Number of Drones: %d\n", drone_count);

    fprintf(report_file, "Simulation Result: %s\n\n", collision_detected ? "FAILED" : "PASSED");

    // Escreve informações sobre o estado dos drones

    fprintf(report_file, "-------------------------------------------------------\n");

    fprintf(report_file, "DRONE's STATUS\n\n");

    for (int i = 0; i < drone_count; i++)
    {

        char script_file[256];

        sprintf(script_file, "drone_%d_script.txt", i);

        // Verifica se o drone esteve envolvido em colisões

        bool involved_in_collision = false;

        for (int j = 0; j < collision_count; j++)
        {

            // Verifica se o drone atual está envolvido na colisão
            // Se o drone atual for um dos drones envolvidos na colisão, marca como envolvido
            if (collisions[j].drone1_id == i || collisions[j].drone2_id == i)
            {

                involved_in_collision = true;

                break;
            }
        }

        int steps = 0;

        if (drones[i].active && !max_col)
        {

            // Se o drone estiver ativo e não houver colisões, conta o número de linhas do script

            steps = count_lines(script_file);
        }
        else if (involved_in_collision)
        {

            for (int j = 0; j < collision_count; j++)
            {

                if (collisions[j].drone1_id == i || collisions[j].drone2_id == i)
                {

                    steps = collisions[j].time; // Use o tempo da colisão como o último passo
                    break; // Não precisa verificar mais colisões
                }
            }
        }
        else
        {

            steps = step - 1; 
        }

        fprintf(report_file, "Drone %d:\n", i);

        fprintf(report_file, "  Script: %s\n", script_file);

        fprintf(report_file, "  Total Steps: %d\n", steps);

        // Determina o status do drone

        const char *status;

        if (drones[i].active && !max_col)
        {

            status = "Completed Successfully!";
        }
        else if (involved_in_collision)
        {

            status = "Terminated (Collision)!";
        }
        else
        {

            status = "Incomplete!";
        }

        fprintf(report_file, "  Status: %s\n", status);

        fprintf(report_file, "  Final Position: (%.2f, %.2f, %.2f)\n\n",

                drones[i].x, drones[i].y, drones[i].z);
    }

    // Escreve informações sobre colisões

    if (collision_detected)
    {

        fprintf(report_file, "-------------------------------------------------------\n");

        fprintf(report_file, "COLLISION(S) DETAILS\n\n");

        fprintf(report_file, "Total Number of Collisions: %d\n\n", collision_count);

        for (int i = 0; i < collision_count; i++)
        {

            fprintf(report_file, "Collision %d:\n", i + 1);

            fprintf(report_file, "  Drones Involved: %d and %d\n",

                    collisions[i].drone1_id, collisions[i].drone2_id);

            fprintf(report_file, "  Time: %.2f seconds\n", collisions[i].time);

            fprintf(report_file, "  Distance between them: %.2f meters\n", collisions[i].distance);

            fprintf(report_file, "  Drone %d Position: (%.2f, %.2f, %.2f)\n",

                    collisions[i].drone1_id,

                    collisions[i].x1, collisions[i].y1, collisions[i].z1);

            fprintf(report_file, "  Drone %d Position: (%.2f, %.2f, %.2f)\n\n",

                    collisions[i].drone2_id,

                    collisions[i].x2, collisions[i].y2, collisions[i].z2);
        }
    }

    // Escreve recomendações

    fprintf(report_file, "-------------------------------------------------------\n");

    fprintf(report_file, "RECOMMENDATIONS\n\n");

    if (collision_detected)
    {

        fprintf(report_file, "The figure is NOT safe to use.\nPlease modify the drone paths to avoid collisions.\n");

        // Sugestões para evitar colisões

        fprintf(report_file, "Consider adjusting the paths of the following drones:\n");

        for (int i = 0; i < collision_count; i++)
        {

            fprintf(report_file, "- Drones %d and %d (collided at time %.2f)\n",

                    collisions[i].drone1_id, collisions[i].drone2_id, collisions[i].time);
        }
    }
    else
    {

        fprintf(report_file, "The figure is safe to use.\nAll drones completed their paths without collisions.\n");
    }

    fclose(report_file);

    printf("Simulation report generated: %s\n", REPORT_FILENAME);
}

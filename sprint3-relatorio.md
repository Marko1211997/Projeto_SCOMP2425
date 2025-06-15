# Sistema de Simulação de Drones - Sprint 3

# Diagrama de Componentes

## Arquitetura do Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                    PROCESSO PRINCIPAL                        │
│                        (main.c)                             │
└─────────────────┬───────────────┬───────────────┬───────────┘
                  │               │               │
                  ▼               ▼               ▼
        ┌─────────────────┐ ┌─────────────┐ ┌─────────────────┐
        │   Initialize    │ │    Start    │ │    Cleanup      │
        │   Simulation    │ │ Simulation  │ │   Simulation    │
        └─────────┬───────┘ └──────┬──────┘ └─────────────────┘
                  │                │
                  ▼                ▼
        ┌─────────────────┐ ┌─────────────────────────────────┐
        │ Setup Resources │ │        Fork Processes           │
        │ • Shared Memory │ │        Create Threads           │
        │ • Semaphores    │ │                                 │
        │ • Signal Handle │ │                                 │
        └─────────────────┘ └─────────────┬───────────────────┘
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
                    ▼                     ▼                     ▼
        ┌─────────────────────┐ ┌─────────────────┐ ┌─────────────────┐
        │   DRONE PROCESSES   │ │     THREADS     │ │ SHARED RESOURCES│
        │                     │ │                 │ │                 │
        │ ┌─────────────────┐ │ │ ┌─────────────┐ │ │ ┌─────────────┐ │
        │ │   Drone 0       │ │ │ │ Collision   │ │ │ │   Shared    │ │
        │ │   Process       │ │ │ │ Detection   │ │ │ │   Memory    │ │
        │ └─────────────────┘ │ │ │   Thread    │ │ │ └─────────────┘ │
        │                     │ │ └─────────────┘ │ │                 │
        │ ┌─────────────────┐ │ │                 │ │ ┌─────────────┐ │
        │ │   Drone 1       │ │ │ ┌─────────────┐ │ │ │ Semaphores  │ │
        │ │   Process       │ │ │ │   Report    │ │ │ │ • Barrier   │ │
        │ └─────────────────┘ │ │ │ Generation  │ │ │ │ • Phase     │ │
        │                     │ │ │   Thread    │ │ │ │ • Individual│ │
        │        ...          │ │ └─────────────┘ │ │ └─────────────┘ │
        │                     │ │                 │ │                 │
        │ ┌─────────────────┐ │ │                 │ │ ┌─────────────┐ │
        │ │   Drone N       │ │ │                 │ │ │   Signals   │ │
        │ │   Process       │ │ │                 │ │ │ • SIGINT    │ │
        │ └─────────────────┘ │ │                 │ │ │ • SIGTERM   │ │
        └─────────────────────┘ └─────────────────┘ │ │ • SIGUSR1   │ │
                    │                     │         │ └─────────────┘ │
                    │                     │         └─────────────────┘
                    ▼                     ▼                     ▲
        ┌──────────────────────┐ ┌─────────────────┐            │
        │   SCRIPT FILES       │ │  REPORT FILE    │            │
        │                      │ │                 │            │
        │ ┌──────────────────┐ │ │ ┌─────────────┐ │            │
        │ │drone_0_script.txt│ │ │ │ simulation_ │ │            │
        │ │drone_1_script.txt│ │ │ │ report.txt  │ │            │
        │ │      ...         │ │ │ └─────────────┘ │            │
        │ │drone_N_script.txt│ │ └─────────────────┘            │
        │ └──────────────────┘ │                                │
        └──────────────────────┘                                │
                    │                                           │
                    └───────────────────────────────────────────┘
                              (Comunicação IPC)
```

## Exemplo de Script de Movimento

### Formato: `drone_0_script.txt`
```
1.0 2.0 1.0 0.0
2.0 1.0 0.0 0.0
3.0 0.0 1.0 0.0
4.0 -1.0 0.0 0.0
5.0 -1.0 -1.0 0.0
6.0 -1.0 -1.0 0.0
```

**Formato**: `tempo dx dy dz`
- **tempo**: Timestamp do movimento (segundos)
- **dx, dy, dz**: Deslocamentos nos eixos X, Y, Z (metros)

## Componentes Detalhados

### **PROCESSO PRINCIPAL**
- **Função**: Coordenação geral da simulação
- **Responsabilidades**:
  - Inicialização do sistema
  - Gestão de recursos IPC
  - Controlo do loop de simulação
  - Limpeza final

### **THREADS**
#### **Collision Detection Thread**
- **Função**: Detecção de colisões em tempo real
- **Operações**:
  - Cálculo de distâncias entre drones
  - Identificação de colisões (< 1.0m)
  - Terminação de drones em colisão

#### **Report Generation Thread**
- **Função**: Geração de relatórios
- **Operações**:
  - Processamento de dados de colisão
  - Criação de relatório final
  - Análise de resultados

### **SHARED RESOURCES**
#### **Shared Memory**
- **Estrutura**: `SharedMemory`
- **Conteúdo**:
  - Array de drones (`Drone drones[MAX_DRONES]`)
  - Array de colisões (`Collision collisions[MAX_COLLISIONS]`)
  - Variáveis de controlo e sincronização
  - Mutexes e variáveis de condição

#### **Semaphores**
- **Barrier Semaphore**: Sincronização de passos
- **Phase Semaphore**: Controlo de fases
- **Individual Semaphores**: Um por drone (controlo individual)

## Fluxo de Comunicação

### **Inter-Process Communication (IPC)**
```
Processo Principal  ←→  Shared Memory  ←→  Drone Processes
       ↕                     ↕                    ↕
   Semaphores        Mutex/Conditions        Semaphores
       ↕                     ↕                    ↕
    Threads          Signal Handling         Script Files
```

## Descrição da Abordagem por User Story

### US361 - Ambiente de Simulação Híbrido
- **Memória Partilhada**: `shm_open()` + `mmap()`
- **Processos Filhos**: `fork()` para cada drone
- **Sincronização**: Semáforos individuais por drone

### US362 - Threads Funcionais
- **Collision Detection Thread**: Detecção contínua de colisões
- **Report Generation Thread**: Geração de relatórios em tempo real
- **Sincronização**: Mutexes e variáveis de condição

### US363 - Notificação por Condition Variables
- **Detecção**: Cálculo de distâncias entre drones
- **Notificação**: `pthread_cond_signal()` para Report Thread
- **Thread-Safety**: Mutexes para acesso seguro aos dados

### US364 - Sincronização Passo-a-Passo
- **Semáforos Individuais**: Um por drone (`drone_sem[i]`)
- **Barreira**: Sincronização de conclusão (`barrier_sem`)

### US365 - Relatório Final
- **Agregação**: Dados da memória partilhada
- **Conteúdo**: Estados dos drones, colisões, validação
- **Armazenamento**: Ficheiro `simulation_report.txt`

### Correções feitas do sprint passado baseado na defesa do mesmo
- **Drone Script**: Em vez de indicar a coordenada onde vai estar, indica quando se move em cada coordenada em cada tempo
- **Signals**: Uso de sigaction em vez de signal

### Problemas encontrados e solução
- **Semaphores**: Inicialmente tinhamos um semaforo para o Step, no inicio de cada Step mandava X (numero de drones ativos) sinais, porem acontecia que um ou mais drones acabavam muito
                    rapido o processo então iniciavam novamente, fazendo assim 2 Steps num só. Solução foi criar um semaforo para cada drone, assim só manda 1 sinal a cada drone expecificamente garantindo que só fazem um Step de cada vez
                  Ao testar o programa ao ocurrer um erro não chegava a parte de dar cleanup aos semaforos, então a solução provisória foi de criar um script a parte para limpar os semaforos
- **Compilação do projeto**: Alguma intreferencia entre semaforos e processos, que com o waitpid(...) ao passar o limite de colizoeos o programa trava - Solução encontrada mas nao
                                a melhor, decerteza, foi comentar a linha waipid(...), parte negativa é a existencia de processos zombies, para a continuação do codigo foi usado um scrip a parte que retirava esses processos zombies para poder compilar o projeto 

## Autoavaliação de Compromisso

|        Nome        | Compromisso (%) | Auto-avaliação | 
|:------------------:|:---------------:|:--------------:|
| Francisco Monteiro |       80        |       17       | 
|    Marco Santos    |       90        |       17       | 
|    Rui Queirós     |       80        |       17       |  
|   Gonçalo Sousa    |       80        |       17       |
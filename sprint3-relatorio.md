# Diagrama de Componentes - Sistema de Simulação de Drones

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
        │ • Shared Memory │ │      Create Threads             │
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
        ┌─────────────────────┐ ┌─────────────────┐             │
        │   SCRIPT FILES      │ │  REPORT FILE    │             │
        │                     │ │                 │             │
        │ ┌─────────────────┐ │ │ ┌─────────────┐ │             │
        │ │   drone0.txt    │ │ │ │ simulation_ │ │             │
        │ │   drone1.txt    │ │ │ │ report.txt  │ │             │
        │ │      ...        │ │ │ └─────────────┘ │             │
        │ │   droneN.txt    │ │ └─────────────────┘             │
        │ └─────────────────┘ │                                 │
        └─────────────────────┘                                 │
                    │                                           │
                    └───────────────────────────────────────────┘
                              (Comunicação IPC)
```

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



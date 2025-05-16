# User Story BreakDown (SCOMP 24/25) - 2DC

## Diagrama de Componentes

<pre lang="markdown">
+---------------------------------------------+
|                Main Process                 |
|                 (main.c)                    |-----------+
+-----+----------------+----------------+-----+           |
      |                |                |                 |
      v                v                v                 v
+-------------+ +-------------+ +----------------+ +-------------+
| Initialize  | |    Start    | |   Generate     | |   Cleanup   |
| Simulation  | | Simulation  | |   Report       | | Simulation  |
+------+------+ +------+------+ +-------+--------+ +------+------+
       |               |                |                |
       v               v                v                v
+-------------+ +-------------+ +----------------+ +-------------+
| Read Figure | | Fork        | | Write          | | Send        |
| File        | | Processes   | | Simulation     | | Termination |
+------+------+ +------+------+ | Report         | | Signals     |
       |               |        +----------------+ +------+------+
       v               |                                  |
+-------------+        |                                  v
| Create Drone|        |                           +-------------+
| Structures  |        |                           | Wait for    |
+------+------+        |                           | Processes   |
       |               |                           +------+------+
       v               |                                  |
+-------------+        |                                  v
| Create Pipes|        |                           +-------------+
+-------------+        |                           | Close Pipes |
                       |                           +-------------+
                       v
       +---------------+---------------+
       |                               |
       v                               v
+-------------+                +----------------+
| Parent      |<--Pipes------->| Child Processes|
| Process     |<--Signals----->| (Drones)       |
+------+------+                +-------+--------+
       |                               |
       v                               |
+-------------+                        |
| Read        |                        |
| Positions   |                        |
+------+------+                        |
       |                               |
       v                               |
+-------------+                        |
| Check       |                        |
| Collisions  |                        |
+------+------+                        |
       |                               |
       v                               |
+-------------+                        |
| Terminate   |                        |
| Colliding   |                        |
| Drones      |                        |
+-------------+                        |
                                       |
                                       v
                       +---------------+---------------+
                       |               |               |
                       v               v               v
               +-------------+ +-------------+ +-------------+
               | Drone       | | Drone       | | Drone       |
               | Process 0   | | Process 1   | | Process N   |
               +------+------+ +------+------+ +------+------+
                      |               |               |
                      v               v               v
               +-------------+ +-------------+ +-------------+
               | Read        | | Read        | | Read        |
               | Script 0    | | Script 1    | | Script N    |
               +-------------+ +-------------+ +-------------+
</pre>

## Exemplo de Script de Movimento

Conteúdo do 'drone_0_script.txt':

| Tempo (em segundos) |  X  |  Y  |  Z   |
|:-------------------:|:---:|:---:|:----:|
|         1.0         | 1.0 | 1.0 | 10.0 |
|         2.0         | 2.0 | 2.0 | 10.0 |
|         3.0         | 3.0 | 3.0 | 10.0 | 
|         4.0         | 4.0 | 4.0 | 10.0 |
|         5.0         | 5.0 | 5.0 | 10.0 |
|         6.0         | 5.0 | 5.5 | 10.0 |


## Abordagem e Implementação

#### US261 - Initiate simulation for a figure

- Ler o "figure.txt" que contém ficheiros de script e posições iniciais dos drones. 
- Criação de um processo filho para cada drone, usando o fork().
- Cada drone lê o seu respetivo script e executa os movimentos definidos. 
- Comunicação com o pai via pipe().

#### US262 - Capture and process drone movements

- Drones enviam posições com o write().
- Pai lê com read(), atualiza o array com o estado dos drones. 
- Cada entrada contém timestamp, posição (X, Y, Z) e o ID do drone.


#### US263 - Detect drone collisions in real time

- Após cada leitura, calcula-sea distância entre drones. 
- Se a distância for menor que 1.0, é registada uma colisão e enviados SIGTERM.
- Guarda-se o tempo e o ID dos drones envolvidos. 


#### US264 - Synchronize drone execution with a time step

- Simulação avança a cada "usleep(100000)" (100ms). 
- Cada drone lê a linha do script e aguarda o tempo relativo. 
- Sincronização garantida por temporização dos processos.


#### US265 - Generate a simulation report

- Criação do ficheiro "simulation_report.txt" com os seguintes atributos:
    - Número de drones
    - Estado final (executado ou colisão)
    - Timestamps de colisão 
    - Resultado da simulação (sucesso ou falha)

    
## Auto-avaliação de compromisso

|        Nome        | Compromisso (%) | Auto-avaliação | 
|:------------------:|:---------------:|:--------------:|
| Francisco Monteiro |       80        |       15       | 
|    Marco Santos    |       90        |       15       | 
|    Rui Queirós     |       80        |       15       |  
|   Gonçalo Sousa    |       80        |       15       |

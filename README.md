# Projeto Culling Games (cg)

Este documento fornece um tutorial sobre como construir, executar e interagir
com o projeto Culling Games ROS 2.

## 1. Construindo o Workspace

Antes de executar qualquer parte do projeto, você precisa construir os pacotes.
Navegue até a raiz do workspace e execute:

```bash
colcon build
```

Este comando irá compilar todos os pacotes (`cg`, `cg_interfaces`, `cg_teleop`).
Lembre-se de "source" o workspace em qualquer novo terminal que você abrir:

```bash
source install/setup.bash
```

## 2. Executando o Jogo

O jogo principal é uma janela Pygame que exibe o labirinto e o movimento do
robô.

Para iniciar o jogo, execute o seguinte comando em um terminal:

```bash
ros2 run cg maze
```

### Opções de Carregamento do Labirinto

Você pode especificar como o labirinto é carregado usando argumentos adicionais:

*   **Carregar um Labirinto Aleatório (Padrão):** Se nenhum argumento for fornecido, o jogo selecionará um labirinto aleatório do diretório `src/cg/maps`.
    ```bash
    ros2 run cg maze
    ```
*   **Carregar um Labirinto Específico:** Para carregar um labirinto pelo nome do arquivo (por exemplo, `test.csv`):
    ```bash
    ros2 run cg maze -- --map test.csv
    ```
*   **Gerar um Novo Labirinto:** Para gerar um novo labirinto aleatório e usá-lo imediatamente (esta opção tem precedência sobre `--map`):
    ```bash
    ros2 run cg maze -- --generate
    ```

## 3. Controlando o Robô

Existem duas maneiras de controlar o robô: usando o nó de teleoperação fornecido
ou enviando chamadas de serviço.

### Método A: Usando o Nó de Teleoperação por Teclado

Esta é a maneira mais fácil de jogar.

1.  Em um **terminal separado** (enquanto o comando `ros2 run cg maze` ainda
    estiver em execução), inicie o nó de teleoperação:
    ```bash
    ros2 run cg_teleop teleop_keyboard
    ```
2.  O terminal exibirá as teclas de atalho. Use as seguintes teclas neste
    terminal para mover o robô na janela do jogo:
    *   **Cima:** `w`, `k`, ou a tecla Seta para Cima
    *   **Baixo:** `s`, `j`, ou a tecla Seta para Baixo
    *   **Esquerda:** `a`, `h`, ou a tecla Seta para Esquerda
    *   **Direita:** `d`, `l`, ou a tecla Seta para Direita

### Método B: Enviando Chamadas de Serviço Manuais

Você também pode enviar comandos de movimento individuais usando o serviço
`/move_command`. Isso é útil para scripts ou depuração.

Para mover o robô um passo, use o comando `ros2 service call`. Por exemplo, para
mover para cima:

```bash
ros2 service call /move_command cg_interfaces/srv/MoveCmd "{direction: 'up'}"
```

Substitua `'up'` por `'down'`, `'left'` ou `'right'` para outras direções.

## 4. Sensoriamento do Ambiente

O robô publica continuamente seus arredores imediatos em um tópico. Isso simula
dados de sensores, mostrando o que está nas 8 células adjacentes (incluindo
diagonais).

*   **Tópico:** `/culling_games/robot_sensors`
*   **Tipo de Mensagem:** `cg_interfaces/msg/RobotSensors`

Para ver esses dados em tempo real, abra um novo terminal e execute:

```bash
ros2 topic echo /culling_games/robot_sensors
```

Você verá um fluxo de mensagens mostrando o que está nas células `up`, `down`,
`left`, `right`, `up_left`, etc., em relação ao robô.

## 5. Reiniciando o Jogo

O serviço `/reset` permite reiniciar o tabuleiro do jogo. Ele suporta dois
modos.

### Reiniciando o Labirinto Atual

Se você quiser tentar o *mesmo* labirinto novamente desde o início.

*   **Com Teleoperação:** Pressione a tecla `r` no terminal `cg_teleop`.
*   **Comando Manual:**
    ```bash
    ros2 service call /reset cg_interfaces/srv/Reset "{is_random: false}"
    ```

### Carregando um Novo Labirinto Aleatório

Se você quiser um novo desafio com um labirinto novo e selecionado
aleatoriamente.

*   **Com Teleoperação:** Pressione a tecla `n` no terminal `cg_teleop`.
*   **Comando Manual:**
    ```bash
    ros2 service call /reset cg_interfaces/srv/Reset "{is_random: true}"
    ```
A resposta do serviço informará o nome do arquivo do novo labirinto que foi
carregado.

## 6. Obtendo os Dados Completos do Labirinto

Se você quiser obter o layout de todo o labirinto atual (por exemplo, para
construir um mapa externo), você pode usar o serviço `/get_map`.

```bash
ros2 service call /get_map cg_interfaces/srv/GetMap
```

Isso retornará uma representação "achatada" da grade do labirinto e suas
dimensões.

# Guia de Execução dos Solucionadores (Parte 1 e Parte 2)

Este documento explica a ordem correta e os comandos necessários para rodar as soluções desenvolvidas (`parte1` e `parte2`) no projeto Culling Games.

## Pré-requisitos

Certifique-se de que você está na raiz do seu workspace (ex: `~/Documents/Github/ros-pond`) e que o projeto foi compilado.

1.  **Compile o projeto:**
    ```bash
    colcon build
    ```

2.  **Configure o ambiente (em CADA novo terminal que abrir):**
    ```bash
    source install/setup.bash
    ```

---

## Passo 1: Iniciar a Simulação

Antes de rodar qualquer solucionador, o ambiente do jogo (o labirinto) precisa estar rodando.

1.  Abra um terminal.
2.  Execute o comando:
    ```bash
    ros2 run cg maze
    ```
    *Isso abrirá a janela do Pygame com o labirinto.*

---

## Passo 2: Rodar a Parte 1 (Navegação Reativa)

A Parte 1 é responsável por uma navegação mais simples/reativa (ou o que foi definido no escopo da parte 1).

1.  Mantenha a simulação rodando no primeiro terminal.
2.  Abra um **segundo terminal**.
3.  Configure o ambiente: `source install/setup.bash`
4.  Execute o nó da Parte 1:
    ```bash
    ros2 run cg_solver parte1
    ```

---

## Passo 3: Rodar a Parte 2 (Mapeamento e Busca)

A Parte 2 implementa a exploração autônoma (DFS) e busca de caminho otimizado (BFS).

1.  Certifique-se de que a simulação está rodando (se necessário, reinicie-a para ter um estado limpo).
2.  Abra um **terminal** (pode usar o mesmo da Parte 1, mas certifique-se de ter encerrado o processo anterior com `Ctrl+C`).
3.  Configure o ambiente: `source install/setup.bash`
4.  Execute o nó da Parte 2:
    ```bash
    ros2 run cg_solver parte2
    ```

## Para rodar em mapas gerados aleatóriamente

Você precisará de pelo menos 2 terminais simultâneos:

| Terminal | Comando | Função |
| :--- | :--- | :--- |
| **Terminal 1** | `ros2 run cg maze -- --generate` | Roda o Jogo/Simulador com mapa gerado aleatoriamente |
| **Terminal 2** | `ros2 run cg_solver parte1` **OU** `parte2` | Roda o seu código de controle |


# Video

Acesse o vídeo [aqui](https://drive.google.com/file/d/1Z7cnPW_pn_STt-sLLercXmn6gcvtnfKY/view?usp=sharing).

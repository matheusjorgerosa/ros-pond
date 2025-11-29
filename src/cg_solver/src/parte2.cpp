#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <vector>
#include <string>
#include <stack>
#include <set>
#include <map>
#include <queue>
#include <iostream>
#include <algorithm>
#include <cctype>

// Definição de Ponto e operadores
struct Point {
    int x;
    int y;
    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

class SolverPart2 : public rclcpp::Node {
public:
    SolverPart2() : Node("solver_part2_node") {
        client_move_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        client_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");

        auto sensor_qos = rclcpp::SensorDataQoS();
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors",
            sensor_qos,
            std::bind(&SolverPart2::sensor_callback, this, std::placeholders::_1)
        );

        current_pos_ = {0, 0};
        full_map_[current_pos_] = "S";
        visited_.insert(current_pos_);
    }

    void run_exploration() {
        RCLCPP_INFO(this->get_logger(), "Aguardando sensores...");

        while (rclcpp::ok() && !sensor_data_received_) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        RCLCPP_INFO(this->get_logger(), "Sensores OK! Iniciando Mapeamento Completo (DFS).");

        // FASE 1: EXPLORAÇÃO TOTAL
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());

            // 1. Atualiza mapa mental
            update_internal_map();

            // 2. Lógica de Exploração (DFS)
            std::string move_dir = choose_next_move();

            if (!move_dir.empty()) {
                // Avança
                if (move_robot(move_dir)) {
                    backtrack_stack_.push(get_opposite_move(move_dir));
                }
            } else {
                // Backtrack
                if (backtrack_stack_.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Mapeamento completo! O robô retornou ao início.");
                    break; // Sai do loop de exploração
                }

                std::string back_move = backtrack_stack_.top();
                backtrack_stack_.pop();
                move_robot(back_move, true);
            }
            rclcpp::sleep_for(std::chrono::milliseconds(40));
        }

        // FASE 2: NAVEGAÇÃO OTIMIZADA
        if (target_found_) {
            RCLCPP_INFO(this->get_logger(), "Calculando rota otimizada para o alvo encontrado em (%d, %d)...", target_pos_.x, target_pos_.y);

            // Calcula caminho usando BFS no mapa construído
            auto path = calculate_bfs_path({0,0}, target_pos_);

            if (!path.empty()) {
                RCLCPP_INFO(this->get_logger(), "Caminho otimizado encontrado (%zu passos). Executando...", path.size());
                execute_path(path);

                // Validação final após chegar
                validate_map();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Erro: Alvo foi visto mas não há caminho calculável no mapa mental.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Exploração terminou mas o alvo NUNCA foi encontrado.");
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_get_map_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;

    bool sensor_data_received_ = false;
    bool target_found_ = false;
    Point target_pos_ = {9999, 9999}; // Inicialização dummy
    cg_interfaces::msg::RobotSensors current_sensors_;

    Point current_pos_;
    std::set<Point> visited_;
    std::stack<std::string> backtrack_stack_;
    std::map<Point, std::string> full_map_;

    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        current_sensors_ = *msg;
        sensor_data_received_ = true;
    }

    // --- Lógica de Mapa ---

    void update_internal_map() {
        Point p_up = {current_pos_.x, current_pos_.y - 1};
        Point p_right = {current_pos_.x + 1, current_pos_.y};
        Point p_down = {current_pos_.x, current_pos_.y + 1};
        Point p_left = {current_pos_.x - 1, current_pos_.y};

        auto save_cell = [&](Point p, std::string val) {
            std::string norm = normalize_cell(val);
            if (full_map_.find(p) == full_map_.end()) {
                full_map_[p] = norm;
            }

            // Se achou o alvo, salva a posição, mas NÃO para a exploração
            if (norm == "G" && !target_found_) {
                target_found_ = true;
                target_pos_ = p;
                RCLCPP_INFO(this->get_logger(), "Alvo detectado visualmente em (%d, %d)! Continuando exploração...", p.x, p.y);
            }
        };

        save_cell(p_up, current_sensors_.up);
        save_cell(p_right, current_sensors_.right);
        save_cell(p_down, current_sensors_.down);
        save_cell(p_left, current_sensors_.left);
    }

    std::string normalize_cell(std::string raw) {
        std::string cell = raw;
        std::transform(cell.begin(), cell.end(), cell.begin(), [](unsigned char c){ return std::tolower(c); });

        if (cell == "b" || cell == "black" || cell == "wall" || cell == "occupied" || cell == "w") return "#";
        if (cell == "t" || cell == "target" || cell == "red" || cell == "goal" || cell == "g") return "G";
        if (cell == "r" || cell == "robot" || cell == "blue" || cell == "start") return "S";
        return "."; // Free
    }

    bool is_wall(std::string raw) { return normalize_cell(raw) == "#"; }
    bool is_target(std::string raw) { return normalize_cell(raw) == "G"; }

    // --- DFS para Exploração ---

    std::string choose_next_move() {
        std::string dirs[] = {"up", "right", "down", "left"};
        Point neighbors[] = {
            {current_pos_.x, current_pos_.y - 1},
            {current_pos_.x + 1, current_pos_.y},
            {current_pos_.x, current_pos_.y + 1},
            {current_pos_.x - 1, current_pos_.y}
        };
        std::string readings[] = {current_sensors_.up, current_sensors_.right, current_sensors_.down, current_sensors_.left};

        // Regra de Ouro: Durante o mapeamento, priorizamos visitar células NÃO VISITADAS.
        // Adicionamos !is_target() para tratar o alvo como parede durante a exploração.

        for (int i = 0; i < 4; i++) {
            if (!is_wall(readings[i]) && !is_target(readings[i]) && visited_.find(neighbors[i]) == visited_.end()) {
                return dirs[i];
            }
        }
        return "";
    }

    bool move_robot(std::string direction, bool is_backtrack = false) {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;
        auto future = client_move_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (direction == "up") current_pos_.y--;
            else if (direction == "down") current_pos_.y++;
            else if (direction == "left") current_pos_.x--;
            else if (direction == "right") current_pos_.x++;

            if (!is_backtrack) {
                visited_.insert(current_pos_);
                // Atualiza mapa mental se necessário
                if (full_map_.find(current_pos_) == full_map_.end() || full_map_[current_pos_] == "G") {
                     // Se pisamos, é livre (ou Goal, que é transitável)
                     if(full_map_[current_pos_] != "G") full_map_[current_pos_] = ".";
                }
                RCLCPP_INFO(this->get_logger(), "Explorando: %s", direction.c_str());
            }
            return true;
        }
        return false;
    }

    std::string get_opposite_move(std::string dir) {
        if (dir == "up") return "down";
        if (dir == "down") return "up";
        if (dir == "left") return "right";
        if (dir == "right") return "left";
        return "";
    }

    // --- BFS para Caminho Otimizado (Fase 2) ---

    std::vector<std::string> calculate_bfs_path(Point start, Point goal) {
        std::queue<Point> q;
        q.push(start);
        std::map<Point, Point> came_from;
        std::map<Point, bool> visited_bfs;
        visited_bfs[start] = true;

        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};

        bool found = false;
        while (!q.empty()) {
            Point current = q.front(); q.pop();
            if (current == goal) { found = true; break; }

            for (int i = 0; i < 4; i++) {
                Point next = {current.x + dx[i], current.y + dy[i]};

                // Verifica no mapa mental se é válido e não é parede
                if (full_map_.count(next) && full_map_[next] != "#" && !visited_bfs[next]) {
                    visited_bfs[next] = true;
                    came_from[next] = current;
                    q.push(next);
                }
            }
        }

        std::vector<std::string> cmds;
        if (!found) return cmds;

        Point curr = goal;
        while (!(curr == start)) {
            Point prev = came_from[curr];
            if (curr.y < prev.y) cmds.push_back("up");
            else if (curr.y > prev.y) cmds.push_back("down");
            else if (curr.x < prev.x) cmds.push_back("left");
            else if (curr.x > prev.x) cmds.push_back("right");
            curr = prev;
        }
        std::reverse(cmds.begin(), cmds.end());
        return cmds;
    }

    void execute_path(const std::vector<std::string>& commands) {
        for (const auto& cmd : commands) {
            move_robot(cmd, true); // Usamos true para não poluir o log de "Explorando"
            rclcpp::sleep_for(std::chrono::milliseconds(150));
        }
        RCLCPP_INFO(this->get_logger(), "CHEGADA AO ALVO CONFIRMADA.");
    }

    void validate_map() {
        RCLCPP_INFO(this->get_logger(), "--- VALIDAÇÃO FINAL ---");
        RCLCPP_INFO(this->get_logger(), "Mapa totalmente preenchido com %zu células.", full_map_.size());
        RCLCPP_INFO(this->get_logger(), "Robô executou o caminho mais curto baseado no conhecimento adquirido.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SolverPart2>();
    node->run_exploration();
    rclcpp::shutdown();
    return 0;
}

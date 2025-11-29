#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <vector>
#include <string>
#include <queue>
#include <map>
#include <set>
#include <algorithm>
#include <iostream>
#include <cctype> // Para tolower

// Definição de um ponto no Grid
struct Point {
    int x, y;
    bool operator<(const Point& other) const {
        return (x != other.x) ? (x < other.x) : (y < other.y);
    }
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

class SolverPart1 : public rclcpp::Node {
public:
    SolverPart1() : Node("solver_part1_node") {
        client_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        client_move_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    void run() {
        // 1. Pega o mapa
        auto map_data = get_map_from_service();
        if (map_data.empty()) return;

        // 2. Acha Start e Goal
        Point start{-1, -1}, goal{-1, -1};
        find_start_and_goal(map_data, start, goal);

        if (start.x == -1 || goal.x == -1) {
            RCLCPP_ERROR(this->get_logger(), "ALERTA: Robô ou Alvo não encontrados!");
            if (start.x == -1) RCLCPP_ERROR(this->get_logger(), " -> Robô (Start) não encontrado. Procurei por 'r', 'robot', 'blue'.");
            if (goal.x == -1) RCLCPP_ERROR(this->get_logger(), " -> Alvo (Goal) não encontrado. Procurei por 't', 'target', 'red'.");
            return;
        }
        // ---------------------

        RCLCPP_INFO(this->get_logger(), "Mapa OK! Inicio: (%d, %d) -> Alvo: (%d, %d)", start.x, start.y, goal.x, goal.y);

        // 3. Calcula Caminho
        auto path = calculate_bfs_path(map_data, start, goal);

        // 4. Executa
        if (!path.empty()) execute_path(path);
        else RCLCPP_WARN(this->get_logger(), "BFS retornou caminho vazio.");
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_get_map_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_;
    int width_ = 0, height_ = 0;

    std::string to_lower(std::string str) {
        std::transform(str.begin(), str.end(), str.begin(),
            [](unsigned char c){ return std::tolower(c); });
        return str;
    }

    std::vector<std::string> get_map_from_service() {
        while (!client_get_map_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return {};
            RCLCPP_INFO(this->get_logger(), "Aguardando serviço /get_map...");
        }

        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = client_get_map_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            width_ = response->occupancy_grid_shape[1];
            height_ = response->occupancy_grid_shape[0];

            RCLCPP_INFO(this->get_logger(), "Dimensões recebidas: %dx%d", width_, height_);

            std::vector<std::string> grid_2d(height_);
            int index = 0;
            std::set<std::string> tipos_encontrados;

            for (int y = 0; y < height_; y++) {
                std::string row_str = "";
                for (int x = 0; x < width_; x++) {
                    std::string raw_cell = response->occupancy_grid_flattened[index++];
                    std::string cell = to_lower(raw_cell);
                    tipos_encontrados.insert(raw_cell);

                    // Baseado nos logs: [b]=Wall, [f]=Free, [r]=Robot, [t]=Target

                    if (cell == "r" || cell == "robot" || cell == "blue" || cell == "start") {
                        row_str += 'S'; // Start (Robot)
                    }
                    else if (cell == "t" || cell == "target" || cell == "red" || cell == "goal") {
                        row_str += 'G'; // Goal (Target)
                    }
                    else if (cell == "b" || cell == "black" || cell == "wall" || cell == "occupied") {
                        row_str += '#'; // Wall (Black)
                    }
                    else {
                        row_str += '.'; // Free (f, white, etc)
                    }
                }
                grid_2d[y] = row_str;
            }

            std::string log_msg = "Tipos de célula recebidos: ";
            for(auto& t : tipos_encontrados) log_msg += "[" + t + "] ";
            RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());

            return grid_2d;
        }
        return {};
    }

    void find_start_and_goal(const std::vector<std::string>& map, Point& start, Point& goal) {
        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                if (map[y][x] == 'S') start = {x, y};
                if (map[y][x] == 'G') goal = {x, y};
            }
        }
    }

    std::vector<std::string> calculate_bfs_path(const std::vector<std::string>& map, Point start, Point goal) {
        std::queue<Point> fila;
        fila.push(start);
        std::map<Point, Point> came_from;
        std::map<Point, bool> visited;
        visited[start] = true;

        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};

        bool found = false;
        while (!fila.empty()) {
            Point current = fila.front();
            fila.pop();
            if (current == goal) { found = true; break; }

            for (int i = 0; i < 4; i++) {
                Point next = {current.x + dx[i], current.y + dy[i]};
                if (next.x >= 0 && next.x < width_ && next.y >= 0 && next.y < height_) {
                    if (map[next.y][next.x] != '#' && !visited[next]) {
                        visited[next] = true;
                        came_from[next] = current;
                        fila.push(next);
                    }
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
            auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            request->direction = cmd;

            RCLCPP_INFO(this->get_logger(), "Enviando comando: %s", cmd.c_str());

            auto future = client_move_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Falha ao enviar comando de movimento");
            }
            // Aumentei um pouco o delay para ficar bonito de ver na tela
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        RCLCPP_INFO(this->get_logger(), "Missão cumprida!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SolverPart1>();
    node->run();
    rclcpp::shutdown();
    return 0;
}

#include "udp_receiver.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <cstring>

extern "C" {
#include "node_api.h"
}

class GoalPublisherNode {
public:
    GoalPublisherNode(void* dora_context, int udp_port)
        : dora_context_(dora_context)
        , udp_port_(udp_port)
        , udp_receiver_(nullptr) {
    }

    bool initialize() {
        std::cout << "Initializing GoalPublisherNode..." << std::endl;

        udp_receiver_ = std::make_unique<UdpReceiver>(udp_port_);
        if (!udp_receiver_->initialize()) {
            std::cerr << "Failed to initialize UDP receiver" << std::endl;
            return false;
        }

        std::cout << "GoalPublisherNode initialized successfully" << std::endl;
        return true;
    }

    void run() {
        std::cout << "GoalPublisherNode running..." << std::endl;

        while (true) {
            void* event = dora_next_event(dora_context_);
            if (event == nullptr) {
                std::cerr << "Failed to get next event" << std::endl;
                break;
            }

            DoraEventType event_type = read_dora_event_type(event);

            if (event_type == DoraEventType_Input) {
                char* id = nullptr;
                size_t id_len = 0;
                read_dora_input_id(event, &id, &id_len);
                std::string topic_id(id, id_len);

                if (topic_id == "timer") {
                    checkAndPublishGoal();
                }
            }
            else if (event_type == DoraEventType_Stop) {
                std::cout << "Received stop event, exiting..." << std::endl;
                free_dora_event(event);
                break;
            }
            else if (event_type == DoraEventType_Error) {
                std::cerr << "Received error event" << std::endl;
            }

            free_dora_event(event);
        }

        std::cout << "GoalPublisherNode stopped" << std::endl;
    }

private:
    void checkAndPublishGoal() {
        if (!udp_receiver_->hasData()) {
            return;
        }

        auto data = udp_receiver_->receiveData();
        if (!data.has_value()) {
            return;
        }

        const std::string& json_str = data.value();

        if (validateGoalJson(json_str)) {
            publishGoal(json_str);
        }
    }

    bool validateGoalJson(const std::string& json_str) {
        try {
            nlohmann::json json_data = nlohmann::json::parse(json_str);

            if (!json_data.contains("x") || !json_data.contains("y")) {
                std::cerr << "Error: Missing required fields (x, y) in goal JSON" << std::endl;
                return false;
            }

            if (!json_data["x"].is_number() || !json_data["y"].is_number()) {
                std::cerr << "Error: x and y must be numbers" << std::endl;
                return false;
            }

            if (json_data.contains("yaw") && !json_data["yaw"].is_number()) {
                std::cerr << "Error: yaw must be a number" << std::endl;
                return false;
            }

            double x = json_data["x"].get<double>();
            double y = json_data["y"].get<double>();
            double yaw = json_data.value("yaw", 0.0);

            std::cout << "Received valid goal: x=" << x << ", y=" << y << ", yaw=" << yaw << std::endl;
            return true;

        } catch (const nlohmann::json::parse_error& e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error validating goal JSON: " << e.what() << std::endl;
            return false;
        }
    }

    void publishGoal(const std::string& json_str) {
        std::string topic = "goal";

        int result = dora_send_output(
            dora_context_,
            const_cast<char*>(topic.c_str()),
            topic.length(),
            const_cast<char*>(json_str.c_str()),
            json_str.length()
        );

        if (result != 0) {
            std::cerr << "Failed to publish goal, error code: " << result << std::endl;
        } else {
            std::cout << "Published goal to dora topic" << std::endl;
        }
    }

    void* dora_context_;
    int udp_port_;
    std::unique_ptr<UdpReceiver> udp_receiver_;
};

int main() {
    std::cout << "Starting goal_publisher_node..." << std::endl;

    void* dora_context = init_dora_context_from_env();
    if (dora_context == nullptr) {
        std::cerr << "Failed to initialize dora context" << std::endl;
        return 1;
    }

    const int UDP_PORT = 9999;
    GoalPublisherNode node(dora_context, UDP_PORT);

    if (!node.initialize()) {
        std::cerr << "Failed to initialize node" << std::endl;
        free_dora_context(dora_context);
        return 1;
    }

    node.run();

    free_dora_context(dora_context);
    std::cout << "goal_publisher_node exited" << std::endl;
    return 0;
}

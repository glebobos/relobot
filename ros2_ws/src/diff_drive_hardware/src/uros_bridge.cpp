// Copyright 2026 ReloBot Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <arpa/inet.h>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

struct TopicBridge {
    uint8_t id;
    std::string name;
    std::string type;
    bool pico_to_zenoh;
};

static const std::vector<TopicBridge> TOPICS = {
    {0, "/robot_joint_states", "sensor_msgs/msg/JointState", true},
    {1, "/imu", "sensor_msgs/msg/Imu", true},
    {2, "/battery_voltage", "std_msgs/msg/Float32", true},
    {3, "/charger_voltage", "std_msgs/msg/Float32", true},
    {4, "/on_dock", "std_msgs/msg/Bool", true},
    {6, "/robot_joint_commands", "sensor_msgs/msg/JointState", false},
};

static pid_t g_child_pid = -1;

static void sig_handler(int sig) {
    if (g_child_pid > 0) {
        kill(g_child_pid, SIGTERM);
        waitpid(g_child_pid, nullptr, 0);
    }
    _exit(0);
}

static bool read_all(int fd, void* buf, size_t count) {
    size_t total = 0;
    while (total < count) {
        ssize_t n = read(fd, static_cast<char*>(buf) + total, count - total);
        if (n <= 0) {
            return false;
        }
        total += static_cast<size_t>(n);
    }
    return true;
}

static void send_serialized(int sock, uint8_t id, std::shared_ptr<const rclcpp::SerializedMessage> msg) {
    uint32_t len = static_cast<uint32_t>(msg->size());
    uint8_t header[5];
    header[0] = id;
    uint32_t nlen = htonl(len);
    std::memcpy(&header[1], &nlen, 4);

    struct iovec iov[2];
    iov[0].iov_base = header;
    iov[0].iov_len = 5;
    iov[1].iov_base = const_cast<uint8_t*>(msg->get_rcl_serialized_message().buffer);
    iov[1].iov_len = len;

    struct msghdr mh{};
    mh.msg_iov = iov;
    mh.msg_iovlen = 2;
    sendmsg(sock, &mh, MSG_NOSIGNAL);
}

static void run_worker(const std::string& mode, int sock, int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("uros_bridge_" + mode);
    RCLCPP_INFO(node->get_logger(), "Native C++ uros_bridge [%s] online on RMW: %s",
                mode.c_str(), rmw_get_implementation_identifier());

    rclcpp::SensorDataQoS sensor_qos;
    rclcpp::QoS reliable_qos(10);
    reliable_qos.reliable();

    size_t max_id = 0;
    for (const auto& t : TOPICS) {
        if (t.id > max_id) max_id = t.id;
    }
    std::vector<std::shared_ptr<rclcpp::GenericPublisher>> publishers(max_id + 1, nullptr);
    std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions;

    for (const auto& topic : TOPICS) {
        if (mode == "dds") {
            if (topic.pico_to_zenoh) {
                // Subscribe on FastDDS from Pico, forward to Zenoh
                auto sub = node->create_generic_subscription(
                    topic.name, topic.type, sensor_qos,
                    [sock, id = topic.id](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
                        send_serialized(sock, id, msg);
                    });
                subscriptions.push_back(sub);
            } else {
                // Publish on FastDDS to Pico from Zenoh
                publishers[topic.id] = node->create_generic_publisher(topic.name, topic.type, reliable_qos);
            }
        } else { // mode == "zenoh"
            if (topic.pico_to_zenoh) {
                // Publish on Zenoh to robot stack from Pico
                publishers[topic.id] = node->create_generic_publisher(topic.name, topic.type, reliable_qos);
            } else {
                // Subscribe on Zenoh from robot stack, forward to Pico
                auto sub = node->create_generic_subscription(
                    topic.name, topic.type, reliable_qos,
                    [sock, id = topic.id](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
                        send_serialized(sock, id, msg);
                    });
                subscriptions.push_back(sub);
            }
        }
    }

    // Background thread reading from socket and publishing into ROS 2
    std::thread reader_thread([sock, publishers, node]() {
        rclcpp::SerializedMessage msg(4096);
        while (rclcpp::ok()) {
            uint8_t header[5];
            if (!read_all(sock, header, 5)) {
                break;
            }
            uint8_t id = header[0];
            uint32_t nlen = 0;
            std::memcpy(&nlen, &header[1], 4);
            uint32_t len = ntohl(nlen);

            if (len > msg.capacity()) {
                msg.reserve(len * 2);
            }
            if (!read_all(sock, msg.get_rcl_serialized_message().buffer, len)) {
                break;
            }
            msg.get_rcl_serialized_message().buffer_length = len;

            if (id < publishers.size() && publishers[id]) {
                publishers[id]->publish(msg);
            }
        }
    });

    rclcpp::spin(node);

    shutdown(sock, SHUT_RDWR);
    close(sock);
    if (reader_thread.joinable()) {
        reader_thread.join();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--worker-dds" && i + 1 < argc) {
            int sock = std::stoi(argv[i + 1]);
            run_worker("dds", sock, argc, argv);
            return 0;
        }
    }

    // Parent process: runs Zenoh worker, forks child for FastDDS worker
    int fds[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
        perror("socketpair");
        return 1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        return 1;
    }

    if (pid == 0) {
        // Child: FastDDS worker
        close(fds[0]); // close parent end
        setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
        setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/ros2_ws/fastdds_localhost.xml", 1);

        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", fds[1]);
        char* child_args[] = {argv[0], const_cast<char*>("--worker-dds"), fd_str, nullptr};
        execv(argv[0], child_args);
        perror("execv child");
        _exit(1);
    }

    // Parent: Zenoh worker
    close(fds[1]); // close child end
    g_child_pid = pid;
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    setenv("RMW_IMPLEMENTATION", "rmw_zenoh_cpp", 1);
    run_worker("zenoh", fds[0], argc, argv);

    if (g_child_pid > 0) {
        kill(g_child_pid, SIGTERM);
        waitpid(g_child_pid, nullptr, 0);
    }

    return 0;
}

// IN PROGRESS

#include <iostream>
#include <fstream>
#include <zenoh.h>
#include <chrono>
#include <thread>
#include <unordered_map>

std::unordered_map<std::string, size_t> data_counter;
std::ofstream log_file("zenoh_traffic.csv");

void data_callback(const zn_sample_t *sample, void *ctx) {
    static auto start_time = std::chrono::steady_clock::now();
    
    std::string topic = sample->key.key;
    size_t data_size = sample->payload.len;
    auto now = std::chrono::steady_clock::now();
    double elapsed_time = std::chrono::duration<double>(now - start_time).count();

    // Track total data per topic
    data_counter[topic] += data_size;

    // Compute throughput (Bytes per second)
    double throughput = data_counter[topic] / elapsed_time;

    // Log data
    log_file << topic << "," << data_size << "," << throughput << "," << elapsed_time << "\n";
    
    std::cout << "Received " << data_size << " bytes on " << topic
              << " | Throughput: " << throughput << " B/s" << std::endl;
}

int main() {
    zn_properties_t *config = zn_config_default();
    zn_session_t *session = zn_open(config);
    zn_subscription_t *sub = zn_declare_subscriber(session, "**", zn_subinfo_default(), data_callback, NULL);

    std::cout << "Listening to all Zenoh topics...\n";

    log_file << "Topic,Data Size,Throughput (B/s),Time Elapsed (s)\n";

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    zn_undeclare_subscriber(sub);
    zn_close(session);
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class NavigationControl : public rclcpp::Node
{
public:
    NavigationControl()
    : Node("navigation_control"), current_control_mode_(0)
    {
        // Publisher to /cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribers
        // Quality of Service (QoS) 1 è sufficiente per i comandi di velocità
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 1, std::bind(&NavigationControl::nav_callback, this, _1));

        corridor_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_corridor", 1, std::bind(&NavigationControl::corridor_callback, this, _1));

        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/control_mode", 10, std::bind(&NavigationControl::mode_callback, this, _1));

        // RIMOZIONE DEL VECCHIO TIMER: Non serve più per la pubblicazione, ora è immediata.
        // Timer for stable publishing
        // timer_ = this->create_wall_timer(50ms, std::bind(&NavigationControl::publish_loop, this));

        // Lifecycle clients
        collision_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/collision_monitor/change_state");

        docking_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/docking_server/change_state");

        // Memorizza il tempo di avvio per il delay di sicurezza
        startup_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Navigation Control node ready.");
    }

private:
    // ---------------------------------------------------------
    // Internal state
    // ---------------------------------------------------------
    // Rimosse le variabili latest_nav_cmd_ e latest_corridor_cmd_
    int current_control_mode_;   // 0=Nav2, 1=Corridor
    rclcpp::Time startup_time_;  // Tempo di avvio per il delay di sicurezza

    // ---------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr corridor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    
    // RIMOSSO: rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr collision_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr docking_client_;

    // ---------------------------------------------------------
    // Helper: send lifecycle transition
    // ---------------------------------------------------------
    void change_lifecycle_state(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
        uint8_t transition)
    {
        if (!client->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Lifecycle service not available: %s",
                        client->get_service_name());
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        client->async_send_request(request);

        RCLCPP_INFO(this->get_logger(),
                    "Requested transition %d on %s",
                    transition, client->get_service_name());
    }

    // ---------------------------------------------------------
    // Callbacks
    // ---------------------------------------------------------
    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Pubblicazione immediata solo se in modalità Nav2
        if (current_control_mode_ == 0) {
            cmd_vel_pub_->publish(*msg);
        }
    }

    void corridor_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Pubblicazione immediata solo se in modalità Corridor
        if (current_control_mode_ == 1) {
            cmd_vel_pub_->publish(*msg);
        }
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int new_mode = msg->data;
        if (new_mode == current_control_mode_)
            return;
        
        // Safety: pubblica un comando di STOP su /cmd_vel prima di effettuare lo switch
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_INFO(this->get_logger(), "Safety stop published before mode switch.");

        // --- START: PROTEZIONE CONTRO INIZIALIZZAZIONE PREMATURA (10s delay) ---
        const double SAFE_STARTUP_DELAY = 10.0; // 10 secondi di sicurezza
        bool startup_phase = (this->now() - startup_time_).seconds() < SAFE_STARTUP_DELAY;

        if (startup_phase) {
            RCLCPP_WARN(this->get_logger(), 
                        "Startup safety delay active. Skipping lifecycle transition for now (%.2f/%.2f s).",
                        (this->now() - startup_time_).seconds(), SAFE_STARTUP_DELAY);
        }
        // --- END: PROTEZIONE CONTRO INIZIALIZZAZIONE PREMATURA ---


        // Gestione delle Transizioni di Stato
        if (!startup_phase) {
            if (new_mode == 1) {
                // TRANSITION: Nav2 -> Corridor (Disattiva Nav2 ausiliario)
                RCLCPP_INFO(this->get_logger(), "Disabling collision_monitor and docking_server (Corridor mode active)...");
                change_lifecycle_state(collision_client_,lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                change_lifecycle_state(docking_client_,lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

            } else if (new_mode == 0) {
                // TRANSITION: Corridor -> Nav2 (Attiva Nav2 ausiliario)
                RCLCPP_INFO(this->get_logger(), "Enabling collision_monitor and docking_server (Nav2 mode active)...");
                //change_lifecycle_state(collision_client_,lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                //change_lifecycle_state(docking_client_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            }
        }

        // Aggiorna la modalità di controllo solo DOPO aver tentato le transizioni di stato (o averle ignorate)
        current_control_mode_ = new_mode;
        RCLCPP_INFO(this->get_logger(), "Switched control mode to %d (0 = Nav2, 1 = Corridor)",
                    current_control_mode_);
    }

    // ---------------------------------------------------------
    // Stable publisher loop (RIMOSSO, ora usiamo la pubblicazione immediata)
    // ---------------------------------------------------------
    // void publish_loop() {}
};

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationControl>());
    rclcpp::shutdown();
    return 0;
}
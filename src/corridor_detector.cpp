#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

class CorridorDetector : public rclcpp::Node
{
public:
    CorridorDetector() : Node("corridor_detector")
    {
        // 1. Inizializzazione Publisher e Subscriber
        corridor_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/corridor_trigger", 10);
        
        // Sottoscrizione a /cmd_vel per ottenere la velocità effettiva (per la fase START)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CorridorDetector::cmd_vel_callback, this, std::placeholders::_1));

        // Timer di integrazione ad alta frequenza (es. 50ms)
        integration_timer_ = this->create_wall_timer(
            50ms, 
            std::bind(&CorridorDetector::integration_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Corridor Detector initialized (Integrated Velocity START/END).");
        RCLCPP_INFO(this->get_logger(), "START distance: %.1fm (using /cmd_vel). END distance: %.1fm (simulated at %.1f m/s).", 
            START_DISTANCE, END_DISTANCE, CORRIDOR_VELOCITY);
    }

private:
    // Constants
    const double START_DISTANCE = 3.0;  // m (Distanza per entrare nel corridoio)
    const double END_DISTANCE = 10.0;    // m (Distanza per uscire dal corridoio)
    const double CORRIDOR_VELOCITY = 0.3; // m/s (Velocità fissa per la simulazione END)
    
    // State variables
    double current_distance_ = 0.0;
    double last_linear_vel_ = 0.0;
    rclcpp::Time last_time_;
    bool in_corridor_ = false;

    // ROS 2 Components
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_trigger_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr integration_timer_;
    rclcpp::TimerBase::SharedPtr end_timer_ = nullptr; // Inizializzato a nullptr

    /**
     * @brief Aggiorna la variabile di velocità lineare con l'ultimo messaggio ricevuto.
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Usiamo solo la componente lineare X (avanti/indietro)
        last_linear_vel_ = msg->linear.x; 
    }

    /**
     * @brief Callback del timer per l'integrazione della distanza percorsa.
     */
    void integration_callback()
    {
        rclcpp::Time current_time = this->now();
        
        if (last_time_.seconds() != 0.0) // Ignora il primo ciclo
        {
            // Calcola il tempo trascorso (delta_t)
            double dt = (current_time - last_time_).seconds();
            
            // Integrazione: Distanza = Velocità * Tempo (solo se non siamo ancora nel corridoio)
            if (!in_corridor_)
            {
                current_distance_ += std::abs(last_linear_vel_) * dt; // Usa il valore assoluto della velocità
                
                // --- LOGICA START (Ingresso nel corridoio) ---
                if (current_distance_ >= START_DISTANCE)
                {
                    RCLCPP_INFO(this->get_logger(), "Distance %.2f m reached (Target %.1f m).", current_distance_, START_DISTANCE);
                    
                    // 1. Pubblica il segnale di TRUE
                    publish_trigger(true);
                    in_corridor_ = true;
                    
                    // 2. Disattiva il timer di integrazione (non serve più)
                    integration_timer_->cancel();

                    // 3. Calcola e avvia il timer di END
                    // Tempo rimanente per uscire dal corridoio: (END_DISTANCE - START_DISTANCE) / CORRIDOR_VELOCITY
                    double remaining_distance = END_DISTANCE - START_DISTANCE; // 3.0 m
                    double end_time_seconds = remaining_distance / CORRIDOR_VELOCITY; // 3.0 / 0.3 = 10.0 secondi
                    
                    RCLCPP_INFO(this->get_logger(), "Starting END timer for %.2f seconds (simulated remaining %.1fm at %.1f m/s).", 
                        end_time_seconds, remaining_distance, CORRIDOR_VELOCITY);

                    const auto END_DELAY = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::duration<double>(end_time_seconds)
                    );
                    
                    end_timer_ = this->create_wall_timer(
                        END_DELAY, 
                        std::bind(&CorridorDetector::end_trigger_callback, this)
                    );
                }
            }
        }
        
        last_time_ = current_time;
    }

    /**
     * @brief Invia il segnale specificato sul topic /corridor_trigger.
     */
    void publish_trigger(bool state)
    {
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = state;
        corridor_trigger_pub_->publish(bool_msg);

        if (state) {
             RCLCPP_INFO(this->get_logger(), "🚨 TRIGGERED START! Published 'in_corridor: true' on /corridor_trigger.");
        } else {
             RCLCPP_INFO(this->get_logger(), "✅ TRIGGERED END! Published 'in_corridor: false' on /corridor_trigger.");
        }
    }

    /**
     * @brief Callback che si attiva per inviare il segnale FALSE (Fine Corridoio).
     */
    void end_trigger_callback()
    {
        publish_trigger(false);
        
        // Disattiviamo il timer di END
        if (end_timer_) {
            end_timer_->cancel();
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorDetector>());
    rclcpp::shutdown();
    return 0;
}
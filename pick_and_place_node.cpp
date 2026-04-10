#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class UR5PickAndPlace : public rclcpp::Node
{
public:
    UR5PickAndPlace()
    : Node("ur5_pick_and_place_cpp")
    {
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gripper_controller/commands", 10);
        
        arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);
        
        info_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_info", 10);
        
        digital_input_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/digital_input/continue", 10,
            std::bind(&UR5PickAndPlace::digitalInputCallback, this, std::placeholders::_1));
        
        puntos_[1] = {{0.0, -1.57, 1.57, -1.57, -1.57, 0.0}, "PUNTO DE RECOGIDA"};
        puntos_[2] = {{0.8, -1.2, 1.2, -1.57, -1.57, 0.8}, "PUNTO INTERMEDIO"};
        puntos_[3] = {{1.2, -1.0, 1.0, -1.57, -1.57, 1.2}, "PUNTO DE COLOCACION"};
        
        RCLCPP_INFO(this->get_logger(), "=== NODO PICK AND PLACE (C++) INICIALIZADO ===");
    }
    
    void ejecutarRutina()
    {
        enviarMensajeInformativo("INICIANDO RUTINA PICK AND PLACE");
        
        enviarMensajeInformativo("MOVIENDO A " + puntos_[1].nombre);
        moverBrazo(puntos_[1].posiciones);
        cerrarGripper();
        activarSalidaDigital();
        
        esperarEntradaDigital();
        
        enviarMensajeInformativo("MOVIENDO A " + puntos_[2].nombre);
        moverBrazo(puntos_[2].posiciones);
        enviarMensajeInformativo("TRANSPORTANDO OBJETO");
        
        enviarMensajeInformativo("MOVIENDO A " + puntos_[3].nombre);
        moverBrazo(puntos_[3].posiciones);
        abrirGripper();
        activarSalidaDigital();
        
        enviarMensajeInformativo("RUTINA PICK AND PLACE COMPLETADA");
    }

private:
    struct Punto {
        std::vector<double> posiciones;
        std::string nombre;
    };
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr digital_input_sub_;
    
    bool digital_input_received_ = false;
    bool digital_output_ = false;
    std::map<int, Punto> puntos_;
    
    void digitalInputCallback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        digital_input_received_ = true;
        RCLCPP_INFO(this->get_logger(), "ENTRADA DIGITAL: SEÑAL RECIBIDA");
    }
    
    void enviarMensajeInformativo(const std::string& texto)
    {
        auto msg = std_msgs::msg::String();
        msg.data = texto;
        info_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "%s", texto.c_str());
    }
    
    void abrirGripper()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.0};
        gripper_pub_->publish(msg);
        digital_output_ = false;
        RCLCPP_INFO(this->get_logger(), "OPERACION: APERTURA DE GRIPPER");
        std::this_thread::sleep_for(1s);
    }
    
    void cerrarGripper()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.8};
        gripper_pub_->publish(msg);
        digital_output_ = true;
        RCLCPP_INFO(this->get_logger(), "OPERACION: CIERRE DE GRIPPER");
        std::this_thread::sleep_for(1s);
    }
    
    void activarSalidaDigital()
    {
        std::string estado = digital_output_ ? "ACTIVADA" : "DESACTIVADA";
        enviarMensajeInformativo("SALIDA DIGITAL: " + estado);
    }
    
    void moverBrazo(const std::vector<double>& posiciones, double tiempo = 3.0)
    {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };
        
        auto punto = trajectory_msgs::msg::JointTrajectoryPoint();
        punto.positions = posiciones;
        punto.time_from_start.sec = static_cast<int>(tiempo);
        punto.time_from_start.nanosec = static_cast<int>((tiempo - static_cast<int>(tiempo)) * 1e9);
        
        msg.points.push_back(punto);
        arm_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moviendo brazo...");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((tiempo + 0.5) * 1000)));
    }
    
    void esperarEntradaDigital(int timeout = 15)
    {
        digital_input_received_ = false;
        enviarMensajeInformativo("ESPERANDO ENTRADA DIGITAL...");
        RCLCPP_INFO(this->get_logger(), "Simular con: ros2 topic pub /digital_input/continue std_msgs/msg/Empty '{}'");
        
        auto start = std::chrono::steady_clock::now();
        while (!digital_input_received_ && 
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start).count() < timeout) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        
        if (digital_input_received_) {
            enviarMensajeInformativo("SEÑAL RECIBIDA - CONTINUANDO");
        } else {
            enviarMensajeInformativo("TIMEOUT - CONTINUANDO SIN SEÑAL");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5PickAndPlace>();
    
    node->ejecutarRutina();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
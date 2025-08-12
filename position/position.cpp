////// On récupère la position du Geomatic Touch sur les axes x, y et z //////

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>



#ifdef _WIN64
#pragma warning(disable : 4996) //Désactive un avertissement de compilation
#endif

#if defined(WIN32)
# include <windows.h>
#else
# include <string.h>
#endif


#include <iostream>  
#include <cassert>
#include <string>
#include <mutex>  

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

class GTposPublisher : public rclcpp::Node {
public:
    GTposPublisher() : Node("geomagic_touch_position_publisher")
     {
        pos_publisher_= this->create_publisher<geometry_msgs::msg::PointStamped>("/geomagic_touch/position", 10);


        // Initialisation de l'appareil haptique
        hHD = hdInitDevice(HD_DEFAULT_DEVICE); //hHD = handle vers le périphérique haptique inialisé par défaut. En cas d'erreur hdInitDevice échoue silencieusement. 
        if (HD_DEVICE_ERROR(error_info = hdGetError())) //On vérifie si la dernière erreur rencontrée par l’API OpenHaptics lors de l'initialisation du périphérique est une erreur critique et non juste un avertissement.
        {
            RCLCPP_ERROR(this->get_logger(), "Erreur initialisation Geomagic Touch");  
            rclcpp::shutdown(); //Arret du noeud 
            return;
        }

        hdStartScheduler(); //Le scheduler gère en temps réel l’exécution des callbacks liés à la mise à jour des positions sur le dispositif.

        if (HD_DEVICE_ERROR(error_info = hdGetError())) //On vérifie si le scheduler a démarré correctement.
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start scheduler");  
            rclcpp::shutdown(); //Arret du noeud.
            return;
        }

        hpose = hdScheduleAsynchronous(poseCallbackGT, this, HD_MAX_SCHEDULER_PRIORITY);//On programme le callback poseCallbackGT pour qu’il soit appelé périodiquement en temps réel par le scheduler avec la priorité la plus élevée.
    
        
        //Timer ROS2 pour publier à 100 Hz.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GTposPublisher::poseCallbackROS, this)
        );

        RCLCPP_INFO(this->get_logger(), "Geomagic Touch ready to send its position");
    }

    
    ~GTposPublisher() //Destructeur de la classe GTposPublisher
    {
        hdUnschedule(hpose); //Retire du scheduler le callback enregistré dans hpose.
        hdStopScheduler(); //Arrête le scheduler haptique. 
        hdDisableDevice(hHD); //Déconnexion avec le périphérique haptique identifié par hHD.
    }



private:

    rclcpp::TimerBase::SharedPtr timer_;
    HHD hHD;

    hduVector3Dd pos;
    std::mutex pos_mutex; //Sert à protéger l'accès à la variable partagée pos entre les deux threads poseCallbackROS et poseCallbackGT.

    HDSchedulerHandle hpose;
    HDErrorInfo error_info;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_publisher_;


    static HDCallbackCode HDCALLBACK poseCallbackGT(void *data)
    {
        GTposPublisher *self = static_cast<GTposPublisher*>(data);
        
        if (!hdWaitForCompletion(self->hHD, HD_WAIT_CHECK_STATUS)) return HD_CALLBACK_CONTINUE; //Si l’appareil n’est pas prêt, on ne fait rien pendant ce cycle.

        hdBeginFrame(self->hHD); //Indique à l’API qu'on va commencer à interagir avec le bras haptique (ici lire des données).
        {
            std::lock_guard<std::mutex> lock(self->pos_mutex);
            hdGetDoublev(HD_CURRENT_POSITION, self->pos); //Récupère la position actuelle du bras haptique.
        }
        hdEndFrame(self->hHD); //Termine la frame haptique en cours et informe donc le système que toutes les mises à jour (ici position) pour ce cycle de calcul sont terminées.


        //Vérifie s'il y a des erreurs et interromp le callback si une erreur du scheduler est détectée.
        if (HD_DEVICE_ERROR(self->error_info = hdGetError()))
        {
            RCLCPP_WARN(self->get_logger(), "Haptic device error detected");   
 
            if (hduIsSchedulerError(&self->error_info))//si l'erreur vient du scheduler alors on arrête le callback définitivement. 
            {
                return HD_CALLBACK_DONE;
            }
        }
  
        return HD_CALLBACK_CONTINUE; // Indique au scheduler de continuer à appeler le callback.

    }


    void poseCallbackROS() 
    {
        geometry_msgs::msg::PointStamped msg;

        std::lock_guard<std::mutex> lock(pos_mutex);
        msg.point.x = pos[0] / 1000.0;  // mm → m
        msg.point.y = pos[1] / 1000.0;
        msg.point.z = pos[2] / 1000.0;

        pos_publisher_->publish(msg);

    }


   
};
 
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GTposPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

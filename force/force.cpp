////// On envoie une force au Geomatic Touch sur les axes x, y et z //////

// TOPIC : /force_torque_sensor_broadcaster/wrench donné par la commande: ros2 topic list 
// TYPE : geometry_msgs/msg/WrenchStamped donné par la commande: ros2 topic info /force_torque_sensor_broadcaster/wrench

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"


#ifdef _WIN64
#pragma warning(disable:4996) //Désactive un avertissement de compilation
#endif

#if defined(WIN32)
# include <windows.h>
#else
# include <string.h>
#endif

#include <stdio.h>
#include <assert.h>

#include <iostream>  
#include <cassert>
#include <string>
#include <mutex>


#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>


class GTforceSubscriber : public rclcpp::Node
{
public:
    GTforceSubscriber() : Node("Geomagic_Touch_force_applied")
    {
        force_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/force_torque_sensor_broadcaster/wrench", 10, std::bind(&GTforceSubscriber::forceCallbackROS, this,std::placeholders::_1));  
        //placeholder: lier le callback forceCallbackROS (qui prend 1 argument) à son subscriber.


    //Initialisation de l'appareil haptique
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);  //hHD = handle vers le périphérique haptique inialisé par défaut. En cas d'erreur hdInitDevice échoue silencieusement. 
    if (HD_DEVICE_ERROR(error_info = hdGetError())) //On vérifie si la dernière erreur rencontrée par l’API OpenHaptics lors de l'initialisation du périphérique est une erreur critique et non juste un avertissement.
    { 
        RCLCPP_ERROR(this->get_logger(), "Error initialisation Geomagic Touch");
        rclcpp::shutdown(); //Arret du noeud.
        return;
    }
   

    hdEnable(HD_FORCE_OUTPUT); //Autorise l'envoi de forces au dispositif.

    hdStartScheduler(); //Le scheduler gère en temps réel la lecture des données (position, vitesse...) et l'application des forces.

    if (HD_DEVICE_ERROR(error_info = hdGetError())) //On vérifie si le scheduler a démarré correctement.
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start scheduler");  
        rclcpp::shutdown(); //Arret du noeud.
        return;
    }

    hforce = hdScheduleAsynchronous(forceCallbackGT, this, HD_MAX_SCHEDULER_PRIORITY); //On programme le callback forceCallbackGT qui rendra les forces à l'appareil pour qu’il soit appelé périodiquement en temps réel par le scheduler avec la priorité la plus élevée.
    RCLCPP_INFO(this->get_logger(), "Geomagic Touch ready to receive force.");

    }

    
    ~GTforceSubscriber() //Destructeur de la classe GTforceSubscriber.
    {   
        hdUnschedule(hforce); //Retire du scheduler le callback enregistré dans hforce. 
        hdStopScheduler(); //Arrêt du scheduler haptique.
        hdDisableDevice(hHD); //Déconnexion avec le périphérique haptique identifié par hHD.
    }


private:

    hduVector3Dd forceSent = {0.0, 0.0, 0.0}; //Forces envoyées par l'utilisateur initialisées à 0;
    HHD hHD;
    HDSchedulerHandle hforce;
    HDErrorInfo error_info;
    std::mutex force_mutex; //Sert à protéger l'accès à la variable partagée forceSent entre les deux threads forceCallbackROS et forceCallbackGT.
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_subscriber_;

    void forceCallbackROS(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(force_mutex); 
        //On récupère les forces envoyées sur le topic /force_torque_sensor_broadcaster/wrench
        forceSent[0] = msg->wrench.force.x;  
        forceSent[1] = msg->wrench.force.y;  
        forceSent[2] = msg->wrench.force.z;  
        RCLCPP_INFO(this->get_logger(), "ROS force received: {x: %f, y: %f, z: %f}", forceSent[0], forceSent[1], forceSent[2]);

           
    }


    static HDCallbackCode HDCALLBACK forceCallbackGT(void *data)
    {
        GTforceSubscriber *self = static_cast<GTforceSubscriber*>(data);

        hdBeginFrame(self->hHD); //Démarre le cycle haptique, signale au scheduler qu’on commence à interagir avec le bras haptique. 
    
        double forceMax;
        hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &forceMax); //Forces maximales selon les axes x,y,z que le robot puisse supporter (ici 0,88 N mais sur https://fr.3dsystems.com/haptics-devices/touch on a 0,75 lbf soit environ 3,3 N)
        double forceMin = -forceMax;
        const char* axes[] = {"x", "y", "z"};

        {
            std::lock_guard<std::mutex> lock(self->force_mutex);
            
           
            for (int i = 0; i < 3; ++i) {
                //On vérifie si les forces selon les 3 axes envoyées par l'utilisateur sont inférieures ou égales à celles supportées au maximum par le robot et si une force dépasse la limite alors on la cape à la valeur maximale autorisée. 
                if ((forceMax < self->forceSent[i])) {
                    RCLCPP_WARN(self->get_logger(),"Force too high on the %s axe: sent %f, max allowed %f, so %f sent to the robot", axes[i], self->forceSent[i], forceMax, forceMax);
                    self->forceSent[i]=forceMax;
                }
                //On vérifie si les forces selon les 3 axes envoyées par l'utilisateur sont supérieures ou égales à celles supportées au minimum par le robot et si une force dépasse la limite on la cape à la valeur minimale autorisée.
                else if ((self->forceSent[i]) < forceMin) {
                    RCLCPP_WARN(self->get_logger(),"Force too low on the %s axe: sent %f, min allowed %f, so %f sent to the robot", axes[i], self->forceSent[i], forceMin, forceMin);
                    self->forceSent[i]= (forceMin);
                }
            }

            
            hdSetDoublev(HD_CURRENT_FORCE, self->forceSent); //Envoie de la force à l'appareil. 
        }

        RCLCPP_INFO(self->get_logger(), "force sent to Geomatic Touch:  {%f, %f, %f}", self->forceSent[0], self->forceSent[1], self->forceSent[2]);

        
        hdEndFrame(self->hHD); //Termine la frame haptique en cours et informe donc le système que toutes les mises à jour (ici forces) pour ce cycle de calcul sont terminées.

        
        //Vérifie s'il y a des erreurs et interromp le callback si une erreur du scheduler est détectée.
        if (HD_DEVICE_ERROR(self->error_info = hdGetError()))
        {
            RCLCPP_WARN(self->get_logger(), "Haptic device error detected");  
            if (hduIsSchedulerError(&self->error_info))//Si l'erreur vient du scheduler alors on arrête le callback définitivement. 
            {
                return HD_CALLBACK_DONE;
            }
        }

        return HD_CALLBACK_CONTINUE; //Indique au scheduler de continuer à appeler le callback.

    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GTforceSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
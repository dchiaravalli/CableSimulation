/* 
 * File:   cable_model.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef CABLE_MODEL
#define CABLE_MODEL

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>


namespace cable_modeller //si usa solitamente il nome del pacchetto dentro cui è messa la libreria
{
    class CableModel    //solitamente si da il nome della classe al file per immediatezza, prime lettere maiuscole attaccate.
    {
        public:

            CableModel();           //costruttore della classe, aggiungi parametri se vuoi che vengano dati dall'esterno quando crei l'oggetto
            virtual ~CableModel();  //distruttore


            //methods

            

            //metodi i/o per variabili (leggere o scrivere variabili direttamente)


        private:

            //class variables, lettere minuscole con "_" tipo: control_reference
            ros::NodeHandle n;
            int cable_points;
            int cable_links;
            float cable_length;
            float cable_mass;
            float linear_spring;
            float bending_spring;
            float twisting_spring;
            std::vector<Eigen::Vector3d> cable; 
            std::vector<Eigen::Vector3d> forces;
            std::vector<Eigen::Vector3d> directors;
            std::vector<Eigen::Vector3d> distances;
            std::vector<float> beta;
            std::vector<float> point_distance;
            std::vector<float> psi;

            //private methods

    };

}
#endif /* CABLE_MODEL */





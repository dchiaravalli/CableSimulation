/* 
 * File:   classe_esempio.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#include "cable_modeller/cable_model.h"

namespace cable_modeller 
{


    CableModel::CableModel(int cable_points, float cable_length, float cable_mass)
    {
        this->cable_points = cable_points;
        this->cable_length = cable_length;
        this->initializeCable();
    }

    CableModel::~CableModel()
    {
    }

    void CableModel::initializeCable()
    {
        this->cable_links = this->cable_points - 1;
        this->point_mass = this->cable_mass/this->cable_points;
        this->link_length = this->cable_length/this->cable_links;
        this->cable.resize(this->cable_points);
        this->forces.resize(this->cable_points);
        this->directors.resize(this->cable_points);
        this->beta.resize(this->cable_points);
        this->psi.resize(this->cable_points);
        this->point_distance.resize(this->cable_points);
        this->distance.resize(this->cable_points);
    }

    void CableModel::resetForces()
    {
        for (int i=0;i<this->cable_points;++i)
        {
            this->forces[i] = Eigen::Vector3d(0,0,0);
        }
    }

    void CableModel::evaluateModelCoefficients()
    {
        //!!!!!!!!!!!!!!!!!evaluate real theta
        float theta = 0;
        //!!!!!!!!!!!!!!!!!
        Eigen::Vector3d a,b;
        for (int i=1;i<this->cable_points;++i)
        {
            this->distances[i] = this->cable[i]-this->cable[i-1];
            this->point_distance[i] = this->distances[i].norm();
            this->directors[i] = this->distances[i]/this->point_distance[i];
        }
        this->beta[1] = atan((this->distances[2].cross(this->distances[1])).norm()/(this->distances[2].transpose()*this->distances[1]));
        for (int i=2;i<this->cable_points-1;++i)
        {
            this->beta[i] = atan((this->distances[i+1].cross(this->distances[i])).norm()/(this->distances[i+1].transpose()*this->distances[i]));
            a = this->distances[i].cross(this->distances[i].cross(this->distances[i-1]));
            b = this->distances[i].cross(this->distances[i+1].cross(this->distances[i]));
            this->psi[i] = atan((a.cross(b)).norm()/(a.transpose()*b)) + theta;
        }
    }

    void CableModel::evaluateLinearSpring()
    {
        for (int i=1;i<this->cable_points - 1;++i)
        {
                this->forces[i] = this->forces[i] - this->linear_spring * (this->point_distance[i] - this->link_length)*this->directors[i];
                this->forces[i] = this->forces[i] + this->linear_spring * (this->point_distance[i+1] - this->link_length)*this->directors[i+1];
        }
    }

    void CableModel::evaluateBendingSpring()
    {
        for (int i=2;i<this->cable_points-2;++i)
        {
            this->forces[i] = this->forces[i] + this->bending_spring * this->beta[i-1] / this->point_distance[i]*this->directors[i].cross(this->directors[i-1].cross(this->directors[i]))/sin(beta[i-1]);
            this->forces[i] = this->forces[i] - this->bending_spring * this->beta[i] / this->point_distance[i]*this->directors[i].cross(this->directors[i].cross(this->directors[i+1]))/sin(beta[i]);
            this->forces[i] = this->forces[i] - this->bending_spring * this->beta[i] / this->point_distance[i+1]*this->directors[i+1].cross(this->directors[i].cross(this->directors[i+1]))/sin(beta[i]);
            this->forces[i] = this->forces[i] + this->bending_spring * this->beta[i+1] / this->point_distance[i+1]*this->directors[i+1].cross(this->directors[i+1].cross(this->directors[i+2]))/sin(beta[i+1]);
        }
    }

    void CableModel::evaluateTwistingSpring()
    {
        for (int i=3;i<this->cable_points-3;++i)
        {
            this->forces[i] = this->forces[i] - this->twisting_spring*this->psi[i-1]/(this->point_distance[i]*sin(this->beta[i]))*this->directors[i-1].cross(this->directors[i])/sin(this->beta[i-1]);
            this->forces[i] = this->forces[i] + this->twisting_spring*this->psi[i]/(this->point_distance[i+1]*sin(this->beta[i]))*this->directors[i].cross(this->directors[i+1])/sin(this->beta[i]);
            this->forces[i] = this->forces[i] + this->twisting_spring*this->psi[i]/(this->point_distance[i]*tan(this->beta[i]))*this->directors[i].cross(this->directors[i+1])/sin(this->beta[i]);
            this->forces[i] = this->forces[i] + this->twisting_spring*this->psi[i]/(this->point_distance[i]*tan(this->beta[i-1]))*this->directors[i-1].cross(this->directors[i])/sin(this->beta[i-1]);
            this->forces[i] = this->forces[i] - this->twisting_spring*this->psi[i+1]/(this->point_distance[i]*sin(this->beta[i]))*this->directors[i].cross(this->directors[i+1])/sin(this->beta[i]);
            this->forces[i] = this->forces[i] - this->twisting_spring*this->psi[i+1]/(this->point_distance[i+1]*tan(this->beta[i]))*this->directors[i].cross(this->directors[i+1])/sin(this->beta[i]);
            this->forces[i] = this->forces[i] - this->twisting_spring*this->psi[i+1]/(this->point_distance[i+1]*tan(this->beta[i+1]))*this->directors[i+1].cross(this->directors[i+2])/sin(this->beta[i+1]);
            this->forces[i] = this->forces[i] + this->twisting_spring*this->psi[i+2]/(this->point_distance[i+1]*sin(this->beta[i+1]))*this->directors[i+1].cross(this->directors[i+2])/sin(this->beta[i+1]);
        }
    }

    void CableModel::evaluateGravity()
    {
        for (int i=1;i<this<this->cable_points-1;++i)
        {
            this->forces[i][2] = this->forces[i][2] - this->point_mass*9.81;
        }
    }


}












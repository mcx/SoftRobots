/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
*                           Plugin SoftRobots                                 *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
******************************************************************************/
#define SOFA_CONTROLLER_ANIMATIONEDITOR_CPP

#include <SoftRobots/component/controller/AnimationEditor.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/type/Quat.h>



namespace softrobots::controller
{

using Quat = sofa::type::Quat<double>;
using sofa::defaulttype::Vec3Types ;
using sofa::defaulttype::Rigid3Types ;


template<>
void AnimationEditor<Rigid3Types>::updateAnimationWithInterpolation(const int startKey,
                                                                     const int endKey)
{
    if(m_state == nullptr)
        return;

    vector<Coord> previousPositions = m_animation[startKey];
    vector<Coord> currentPositions  = m_animation[endKey];

    if (currentPositions.size() != previousPositions.size())
    {
        msg_warning() <<"This component does not handle mechanical state size changes";
        return;
    }

    int nbPositions = m_state->read(sofa::core::vec_id::read_access::position)->getValue().size();
    int nbStep = endKey - startKey;

    for (int i=0; i<nbStep+1; i++)
    {
        vector<Coord> newPositions;
        for (int k=0; k<nbPositions; k++)
        {
            Rigid3Types::CPos direction = currentPositions[k].getCenter() - previousPositions[k].getCenter();
            double distance = direction.norm();
            double step = distance/nbStep*i;
            direction.normalize();
            Rigid3Types::CPos center =  previousPositions[k].getCenter() + direction*step;

            double h =  (double)i/(double)nbStep;
            Quat orientation;
            for(int l=0; l<4; l++)
                orientation[l] = previousPositions[k].getOrientation()[l]*(1.-h) + currentPositions[k].getOrientation()[l]*h;
            orientation.normalize();

            Coord newPosition;
            Rigid3Types::setCPos(newPosition, center);
            Rigid3Types::setCRot(newPosition, orientation);
            newPositions.push_back(newPosition);
        }
        m_animation[startKey+i] = newPositions;
    }
}


template<>
void AnimationEditor<Rigid3Types>::drawTrajectory(const VisualParams* vparams)
{
    vector<Rigid3Types::CPos> points;
    vector<unsigned int> IDSorted = m_keyFramesID;
    unsigned int nbKey = m_keyFramesID.size();
    std::sort(IDSorted.begin(), IDSorted.begin() + nbKey);
    for(unsigned int i=0; i<nbKey; i++)
    {
        for(unsigned int k=0; k<m_animation[i].size(); k++)
        {
            points.push_back(m_animation[IDSorted[i]][k].getCenter());
            vparams->drawTool()->drawFrame(m_animation[m_keyFramesID[i]][k].getCenter(), m_animation[m_keyFramesID[i]][k].getOrientation(), Rigid3Types::CPos(d_drawSize.getValue(),d_drawSize.getValue(),d_drawSize.getValue()));
        }
    }

    vector<Rigid3Types::CPos> lines;
    for(unsigned int i=0; i<points.size()-1; i++)
    {
        lines.push_back(points[i]);
        lines.push_back(points[i+1]);
    }

    vparams->drawTool()->drawLines(lines,d_drawSize.getValue()*2., sofa::type::RGBAColor(0.5,0.5,0.5,1.));
}


////////////////////////////////////////////    FACTORY    ////////////////////////////////////////////
void registerAnimationEditor(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Build an animation from key points motion: \n"
                                   "ctrl+a: add keyframe \n"
                                   "ctrl+d: delete keyframe \n"
                                   "ctrl+c: copy keyframe \n"
                                   "ctrl+v: paste keyframe \n"
                                   "ctrl+x: cut keyframe \n"
                                   "ctrl+w: write animation \n"
                                   "ctrl+m: play/pause animation \n"
                                   "ctrl+(left/right)arrow: move the cursor along the timeline \n"
                                   "ctrl+(pgDn/pgUp): move the cursor to the next/previous keyframe")
    .add< AnimationEditor<Vec3Types> >(true)
    .add< AnimationEditor<Rigid3Types> >());
}

template class SOFA_SOFTROBOTS_API AnimationEditor<Vec3Types>;
template class SOFA_SOFTROBOTS_API AnimationEditor<Rigid3Types>;


} // namespace

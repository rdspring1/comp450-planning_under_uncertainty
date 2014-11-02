/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_PLANNERS_SMR_SMR_
#define OMPL_CONTROL_PLANNERS_SMR_SMR_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <map>

namespace ompl
{

    namespace control
    {

        /** \brief Stochastic Motion Roadmap */
        class SMR : public base::Planner
        {
        public:

            /** \brief Constructor */
            SMR(const SpaceInformationPtr &si);

            virtual ~SMR(void);

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            virtual void clear(void);

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            virtual void setup(void);

            /** \brief Set the number of nodes in the roadmap */
            void setNodes(int N)
            {
                nodes_ = N;
            }

            /** \brief Get the number of nodes in the roadmap */
            int getNodes(void) const
            {
                return nodes_;
            }

            /** \brief Return the number of samples generated per transition */
            int getTransitionStates(void) const
            {
                return trans_;
            }

            /** \brief Set the number of samples taken for each transition */
            void setTransitionStates(int M)
            {
                trans_ = M;
            }
        protected:
            /** \brief Representation of a motion */
            class Motion
            {
            public:

                Motion(void) : state(NULL) {}

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si, int id) : state(NULL), id_(id) {}

                ~Motion(void) {}

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The probability of moving to state T after applying control u - Transition Probability List */
                std::map<int, std::map<int, double>> t; 
                
                /** \brief Unique State ID */
                int id_;
            };
            
            /** \brief Setup Transition Probabilities */
            void setupTransitions(Motion* m);

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr                      controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation                        *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief n states in roadmap */
            int                                            nodes_ = 50000;

            /** \brief m samples per transition */
            int                                            trans_ = 20;

            const double                                   epsilon = 0.0000001;

            const double                                   gamma = 0.000001;
            
            const int                                      actions = 2;

            Motion*                                        obstacle;
        };

    }
}

#endif

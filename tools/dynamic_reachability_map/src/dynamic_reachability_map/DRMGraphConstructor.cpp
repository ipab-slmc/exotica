/*
 * DRMGraphConstructor.cpp
 *
 *  Created on: 4 Dec 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMGraphConstructor.h"

namespace dynamic_reachability_map
{
  DRMGraphConstructor::DRMGraphConstructor()
  {

  }

  DRMGraphConstructor::~DRMGraphConstructor()
  {

  }

  bool DRMGraphConstructor::createGraph(DRMSpace_ptr & space)
  {
    space_ = space;
    space_->edges_.clear();
    std::vector<std::vector<unsigned int>> sample_occups(space_->sample_size_);

    for (unsigned int i = 0; i < space->space_size_; i++)
    {
      space_->volumes_[i].occup_edges.clear();
      for (unsigned int j = 0; j < space_->volumes_[i].occup_samples.size();
          j++)
        sample_occups[space_->volumes_[i].occup_samples[j]].push_back(i);
    }

    for (unsigned long int i = 0; i < space_->sample_size_; i++)
    {
      std::map<double, unsigned long int> knn;
      getKNN(i, knn);
      for (auto &it : knn)
      {
        Edge new_edge(i, it.second, it.first);
        space_->edges_.push_back(new_edge);
      }
    }
    return true;
  }

  void DRMGraphConstructor::getKNN(const unsigned long int index,
      std::map<double, unsigned long int> &knn)
  {
    int k = 10;
    double tol = M_PI / 8 * space_->dimension_;
    std::vector<std::pair<unsigned int, double> > neighbors =
        space_->getNeighborIndices(space_->samples_[index].eff_index, 1);
    for (int i = 0; i < neighbors.size(); i++)
    {
      for (int j = 0;
          j < space_->volumes_[neighbors[i].first].reach_samples.size(); j++)
      {
        unsigned long int tmp_index =
            space_->samples_[space_->volumes_[neighbors[i].first].reach_samples[j]].drake_q;
        if (tmp_index > index)
        {
          double dist = (space_->samples_[index].drake_q - tmp_index).norm();
          if (dist < tol)
          {
            if (knn.size() < k)
              knn[dist] = tmp_index;
            else if (knn.rbegin()->first > dist)
            {
              knn.erase(knn.rbegin()->first);
              knn[dist] = tmp_index;
            }
          }
        }
      }
    }
  }
}


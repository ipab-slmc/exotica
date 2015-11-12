/*
 * DRMSpace.cpp
 *
 *  Created on: 16 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMSpace.h"

namespace dynamic_reachability_map
{

  SpaceBounds::SpaceBounds()
  {

  }

  SpaceBounds::SpaceBounds(double xlow, double xup, double ylow, double yup,
      double zlow, double zup)
      : x_low(xlow), x_upper(xup), y_low(ylow), y_upper(yup), z_low(zlow), z_upper(
          zup)
  {

  }

  bool SpaceBounds::isValid()
  {
    return x_low >= x_upper || y_low >= y_upper || z_low >= z_upper;
  }

  bool SpaceBounds::inBounds(const geometry_msgs::Point &p)
  {
    return p.x >= x_low && p.x <= x_upper && p.y >= y_low && p.y <= y_upper
        && p.z >= z_low && p.z <= z_upper;
  }

  bool SpaceBounds::inBounds(const Eigen::Affine3d &p)
  {
    //The translation part is at (12,13,14)
    return p.data()[12] >= x_low && p.data()[12] <= x_upper
        && p.data()[13] >= y_low && p.data()[13] <= y_upper
        && p.data()[14] >= z_low && p.data()[14] <= z_upper;
  }

  void SpaceBounds::print()
  {
    if (isValid())
      ROS_INFO_STREAM(
          "Valid Space bounds ["<<x_low<<","<<x_upper<<"] ["<<y_low<<","<<y_upper<<"] ["<<z_low<<","<<z_upper<<"]");
    else
      ROS_WARN_STREAM(
          "Invalid Space bounds ["<<x_low<<","<<x_upper<<"] ["<<y_low<<","<<y_upper<<"] ["<<z_low<<","<<z_upper<<"]");
  }

  VolumeBounds::VolumeBounds()
  {

  }

  VolumeBounds::VolumeBounds(const SpaceBounds &bounds,
      double volume_resolution)
  {
    ROS_WARN_STREAM(
        "Space bounds ["<<bounds.x_low<<","<<bounds.x_upper<<"] ["<<bounds.y_low<<","<<bounds.y_upper<<"] ["<<bounds.z_low<<","<<bounds.z_upper<<"] resolution "<<volume_resolution);
    bx = (bounds.x_upper - bounds.x_low) / volume_resolution + 1;
    by = (bounds.y_upper - bounds.y_low) / volume_resolution + 1;
    bz = (bounds.z_upper - bounds.z_low) / volume_resolution + 1;
  }

  bool VolumeBounds::inBounds(const std::vector<int> &p)
  {
    return p.size() == 3 && p[0] < bx && p[1] < by && p[2] < bz;
  }

  SparseBitSets::SparseBitSets()
      : data(NULL), pair_size(0)
  {

  }

  SparseBitSets::~SparseBitSets()
  {
    if (data) delete[] data;
  }

  void SparseBitSets::setFromBoolVector(const std::vector<bool> &bitset)
  {
    std::vector<std::pair<unsigned int, unsigned int>> tmp;
    bool find_first = true;
    unsigned int tmp_first = 0;
    for (unsigned int i = 0; i < bitset.size(); i++)
    {
      if (bitset[i])
      {
        if (find_first)
        {
          unsigned int ii = i;
          tmp_first = ii;
          find_first = false;
        }
      }
      else if (!find_first)
      {
        i--;
        unsigned int ii = i;
        std::pair<unsigned int, unsigned int> tmp_pair(tmp_first, ii);
        tmp.push_back(tmp_pair);
        find_first = true;
      }
    }

    if (!find_first)
    {
      std::pair<unsigned int, unsigned int> tmp_pair(tmp_first,
          bitset.size() - 1);
      tmp.push_back(tmp_pair);
    }

    if (data) delete[] data;
    pair_size = tmp.size();
    data = new SparsePair[pair_size];
    for (int i = 0; i < pair_size; i++)
    {
      data[i].low = tmp[i].first;
      data[i].upper = tmp[i].second;
    }
  }

  std::vector<int> SparseBitSets::getOccupList()
  {
    std::vector<int> tmp;
    for (unsigned int i = 0; i < pair_size; i++)
      for (unsigned int j = 0; j < data[i].upper - data[i].low + 1; j++)
        tmp.push_back(data[i].low + j);
    return tmp;
  }

  Node::Node()
      : isValid(true), q(NULL)
  {
  }

  Node::~Node()
  {
    if (q) delete[] q;
  }

  void Node::invalidate()
  {
    isValid = false;
  }

  Volume::Volume()
      : isFree(true)
  {
  }

  Volume::~Volume()
  {
  }

  Edge::Edge()
      : isValid(false), length(INFINITY), a(0), b(0)
  {

  }

  Edge::Edge(unsigned long int a_, unsigned long int b_, double length_)
      : a(a_), b(b_), length(length_), isValid(true)
  {

  }

  Edge::~Edge()
  {

  }

  DRMSpace::DRMSpace()
      : thread_size_(1)
  {
  }

  DRMSpace::~DRMSpace()
  {
  }

  bool DRMSpace::createSpace(const SpaceBounds &bounds,
      double volume_resolution, const robot_model::RobotModelConstPtr &model,
      const std::string & eff, const std::string & group_name)
  {
    if (volume_resolution <= 0)
    {
      ROS_ERROR_STREAM("Invalid volume resolution "<<volume_resolution);
      return false;
    }
    space_bounds_ = bounds;
    volume_bounds_ = VolumeBounds(space_bounds_, volume_resolution);
    resolution_ = volume_resolution;
    space_size_ = volume_bounds_.bx * volume_bounds_.by * volume_bounds_.bz;
    volumes_.resize(space_size_);
    for (unsigned int i = 0; i < space_size_; i++)
      volume_locks_.push_back(new boost::mutex);
    int cnt = 0;
    int int_x = 0;
    int int_y = 0;
    int int_z = 0;
    double current_x = space_bounds_.x_low + 0.5 * resolution_;
    double current_y = space_bounds_.y_low + 0.5 * resolution_;
    double current_z = space_bounds_.z_low + 0.5 * resolution_;
    for (int z = 0; z < volume_bounds_.bz; z++)
    {
      current_y = space_bounds_.y_low + 0.5 * resolution_;
      int_y = 0;
      for (int y = 0; y < volume_bounds_.by; y++)
      {
        current_x = space_bounds_.x_low + 0.5 * resolution_;
        int_x = 0;
        for (int x = 0; x < volume_bounds_.bx; x++)
        {
          volumes_[cnt].center.x = current_x;
          volumes_[cnt].center.y = current_y;
          volumes_[cnt].center.z = current_z;
          current_x += resolution_;
          int_x++;
          cnt++;
        }
        current_y += resolution_;
        int_y++;
      }
      current_z += resolution_;
      int_z++;
    }

    if (!model->hasLinkModel(eff))
    {
      ROS_ERROR_STREAM("Robot "<<model->getName()<<" does not have link "<<eff);
      return false;
    }
    ps_.reset(new planning_scene::PlanningScene(model));
    eff_ = eff;
    if (!model->hasJointModelGroup(group_name))
    {
      ROS_ERROR_STREAM(
          "Robot "<<model->getName()<<" does not have group "<<group_name);
      return false;
    }
    group_ = model->getJointModelGroup(group_name);

    var_index_.resize(group_->getVariableCount());
    for (int i = 0; i < var_index_.size(); i++)
      var_index_[i] = group_->getVariableIndexList()[i];
    dimension_ = var_index_.size();
    return true;
  }

  void DRMSpace::clear()
  {
    for (unsigned int i = 0; i < space_size_; i++)
    {
      volumes_[i].reach_samples.clear();
      volumes_[i].occup_samples.clear();
      volumes_[i].occup_edges.clear();
    }
    samples_.clear();
  }

  const SpaceBounds & DRMSpace::getSpaceBounds()
  {
    return space_bounds_;
  }

  const VolumeBounds & DRMSpace::getVolumeBounds()
  {
    return volume_bounds_;
  }

  bool DRMSpace::isReachable(const geometry_msgs::Point &p)
  {
    return space_bounds_.inBounds(p);
  }

  double DRMSpace::getResolution()
  {
    return resolution_;
  }

  unsigned int DRMSpace::getSpaceSize()
  {
    return space_size_;
  }

  unsigned long int DRMSpace::getSampleSize()
  {
    return sample_size_;
  }

  int DRMSpace::getDimension()
  {
    return dimension_;
  }

  bool DRMSpace::getVolumeIndex(const geometry_msgs::Point &p,
      unsigned int & index)
  {
    if (!space_bounds_.inBounds(p)) return false;
    unsigned int x = (p.x - space_bounds_.x_low) / resolution_;
    unsigned int y = (p.y - space_bounds_.y_low) / resolution_;
    unsigned int z = (p.z - space_bounds_.z_low) / resolution_;
    index = x + y * volume_bounds_.bx
        + z * volume_bounds_.bx * volume_bounds_.by;
    return index < space_size_;
  }

  bool DRMSpace::getVolumeIndex(const Eigen::Affine3d &p, unsigned int & index)
  {
    if (!space_bounds_.inBounds(p))
    {
      return false;
    }
    unsigned int x = (p.data()[12] - space_bounds_.x_low) / resolution_;
    unsigned int y = (p.data()[13] - space_bounds_.y_low) / resolution_;
    unsigned int z = (p.data()[14] - space_bounds_.z_low) / resolution_;
    index = x + y * volume_bounds_.bx
        + z * volume_bounds_.bx * volume_bounds_.by;
    return index < space_size_;
  }

  std::vector<std::pair<unsigned int, double> > DRMSpace::getNeighborIndices(
      unsigned int index, unsigned int depth)
  {
    std::vector<std::pair<unsigned int, double> > ret;
    if (depth < 1) return ret;

    geometry_msgs::Point c = volumes_[index].center;
    for (double x = -0.5 * resolution_ * depth; x <= 0.5 * resolution_;
        x += 0.5 * resolution_)
      for (double y = -0.5 * resolution_ * depth; y <= 0.5 * resolution_;
          y += 0.5 * resolution_)
        for (double z = -0.5 * resolution_ * depth; z <= 0.5 * resolution_;
            z += 0.5 * resolution_)
        {
          geometry_msgs::Point n;
          n.x = c.x + x;
          n.y = c.y + y;
          n.z = c.z + z;
          unsigned int n_index = 0;
          if (getVolumeIndex(n, n_index) && index != n_index)
          {
            std::pair<unsigned int, double> tmp(n_index,
                sqrt(getDistance(index, n_index)));
            ret.push_back(tmp);
          }
        }
    return ret;
  }

  double DRMSpace::getDistance(unsigned int a, unsigned int b)
  {
    return sqrt(
        pow(volumes_[a].center.x - volumes_[b].center.x, 2)
            + pow(volumes_[a].center.y - volumes_[b].center.y, 2)
            + pow(volumes_[a].center.z - volumes_[b].center.z, 2));
  }

  void DRMSpace::registOccupation(const Node* const &node)
  {
    //TODO
  }

  void DRMSpace::initialiseSamples(int sample_size)
  {
    sample_size_ = sample_size;
    samples_.resize(sample_size);
    for (int i = 0; i < sample_size_; i++)
      samples_[i].q = new float[dimension_];
  }

  void DRMSpace::reserveSamples(int sample_size)
  {
    samples_.clear();
    samples_.reserve(sample_size);
  }

  const Volume & DRMSpace::at(unsigned x, unsigned y, unsigned z) const
  {
    return volumes_[z * volume_bounds_.bz * volume_bounds_.by
        + y * volume_bounds_.by + x];
  }

  const Volume & DRMSpace::at(unsigned index) const
  {
    return volumes_[index];
  }

  Volume & DRMSpace::atNonConst(unsigned x, unsigned y, unsigned z)
  {
    return volumes_[z * volume_bounds_.bz * volume_bounds_.by
        + y * volume_bounds_.by + x];
  }

  Volume & DRMSpace::atNonConst(unsigned int index)
  {
    boost::mutex::scoped_lock(volume_locks_[index]);
    return volumes_[index];
  }

  void DRMSpace::addOccupSample(unsigned int volume_index,
      unsigned long int sample_index)
  {
    boost::mutex::scoped_lock(volume_locks_[volume_index]);
    volumes_[volume_index].occup_samples.push_back(sample_index);
  }

  void DRMSpace::addReachSample(unsigned int volume_index,
      unsigned long int sample_index)
  {
    boost::mutex::scoped_lock(volume_locks_[volume_index]);
    volumes_[volume_index].reach_samples.push_back(sample_index);
  }

  const Node & DRMSpace::getSample(unsigned int index) const
  {
    return samples_[index];
  }

  Node & DRMSpace::getSampleNonConst(unsigned long int index)
  {
    return samples_[index];
  }

  void DRMSpace::setVolumeOccupied(unsigned int index, bool invalidate_samples)
  {
    if (index < space_size_)
    {
      volumes_[index].isFree = false;
      if (invalidate_samples)
      {
        for (unsigned long int i = 0; i < volumes_[index].occup_samples.size();
            i++)
          samples_[volumes_[index].occup_samples[i]].invalidate();
        for (unsigned long int i = 0; i < volumes_[index].occup_edges.size();
            i++)
          edges_[volumes_[index].occup_edges[i]].isValid = false;
      }
    }
  }

  void DRMSpace::setVolumeFree(unsigned int index, bool free_samples)
  {
    if (index < space_size_)
    {
      volumes_[index].isFree = true;
      if (free_samples)
      {
        for (unsigned long int i = 0; i < volumes_[index].occup_samples.size();
            i++)
          samples_[volumes_[index].occup_samples[i]].isValid = true;
        for (unsigned long int i = 0; i < volumes_[index].occup_edges.size();
            i++)
          edges_[volumes_[index].occup_edges[i]].isValid = true;
      }
    }
  }

  std::vector<unsigned int> DRMSpace::getVolumeReachabilities()
  {
    std::vector<unsigned int> ret(space_size_);
    for (unsigned int i = 0; i < space_size_; i++)
    {
      if (volumes_[i].isFree)
      {
        unsigned int tmp = 0;
        for (unsigned long int j = 0; j < volumes_[i].reach_samples.size(); j++)
          if (samples_[volumes_[i].reach_samples[j]].isValid) tmp++;
        ret[i] = tmp;
      }
      else
        ret[i] = 0;
    }

    return ret;
  }

  unsigned long int DRMSpace::CurrentlyReachability(unsigned int index,
      std::vector<unsigned long int> & valid_samples)
  {
    valid_samples.clear();
    valid_samples.reserve(volumes_[index].reach_samples.size());
    for (unsigned long int j = 0; j < volumes_[index].reach_samples.size(); j++)
      if (samples_[volumes_[index].reach_samples[j]].isValid)
        valid_samples.push_back(volumes_[index].reach_samples[j]);
    return valid_samples.size();
  }

  unsigned long int DRMSpace::CurrentlyReachability(unsigned int index,
      const std::vector<unsigned int> &invalid_clusters,
      const std::vector<unsigned int> &invalid_cluster_cells,
      std::vector<unsigned long int> & valid_samples,
      std::vector<std::pair<unsigned int, unsigned int>> & sample_info)
  {
    valid_samples.clear();
    valid_samples.reserve(volumes_[index].reach_samples.size());
    sample_info.clear();
    sample_info.reserve(valid_samples.capacity());
    for (unsigned long int i = 0; i < volumes_[index].reach_clusters.size();
        i++)
    {
      bool valid = true;
      for (unsigned int j = 0; j < invalid_clusters.size(); j++)
        if (invalid_cluster_cells[j] == index && i == invalid_clusters[j])
        {
          valid = false;
          break;
        }
      if (valid)
      {
        for (unsigned int j = 0; j < volumes_[index].reach_clusters[i].size();
            j++)
        {
          valid_samples.push_back(volumes_[index].reach_clusters[i][j]);
          std::pair<unsigned int, unsigned int> tmp_pair(index, i);
          sample_info.push_back(tmp_pair);
        }
      }
    }
    return valid_samples.size();
  }

  void DRMSpace::qArray2Eigen(const float* q, Eigen::VectorXf &eigen)
  {
    eigen.resize(dimension_);
    for (int i = 0; i < dimension_; i++)
      eigen(i) = q[i];
  }
  void DRMSpace::qEigen2Array(const Eigen::VectorXf &eigen, float* q)
  {
    for (int i = 0; i < eigen.rows(); i++)
      q[i] = eigen(i);
  }
  planning_scene::PlanningScenePtr & DRMSpace::getPlanningScene()
  {
    return ps_;
  }

  const robot_state::JointModelGroup *DRMSpace::getGroup()
  {
    return group_;
  }

  void DRMSpace::buildConnectionGraph(double dmax)
  {
    //Get sample occupation
    sample_occupation_.resize(sample_size_);
    for (int i = 0; i < space_size_; i++)
      for (int j = 0; j < volumes_[i].occup_samples.size(); j++)
      {
        sample_occupation_[volumes_[i].occup_samples[j]].push_back(i);
      }

    std::vector<boost::thread*> th(thread_size_);
    th_edges_.resize(thread_size_);
    unsigned int tmp = space_size_ / thread_size_;
    for (unsigned int i = 0; i < thread_size_; ++i)
    {
      std::pair<unsigned int, unsigned int> volumes(i * tmp,
          i == thread_size_ - 1 ? space_size_ : (i + 1) * tmp);
      th[i] = new boost::thread(
          boost::bind(&DRMSpace::buildGraphThreadFn, this, i, volumes, dmax));
    }
    for (unsigned int i = 0; i < thread_size_; ++i)
    {
      th[i]->join();
      delete th[i];
    }

    int edge_size = 0;
    for (unsigned int i = 0; i < thread_size_; ++i)
    {
      ROS_INFO_STREAM("Size "<<i<<" "<<th_edges_[i].size());
      edge_size += th_edges_[i].size();
    }

    ROS_INFO_STREAM("Build graph finished with "<<edge_size<<" edges");
    edges_.resize(edge_size);
    edge_size = 0;
    for (unsigned int t = 0; t < thread_size_; ++t)
    {
      for (unsigned int te = 0; te < th_edges_[t].size(); te++)
      {
        edges_[edge_size] = th_edges_[t][te].first;
        samples_[edges_[edge_size].a].edges.push_back(edge_size);
        samples_[edges_[edge_size].b].edges.push_back(edge_size);
        for (unsigned int teo = 0; teo < th_edges_[t][te].second.size(); teo++)
          volumes_[th_edges_[t][te].second[teo]].occup_edges.push_back(
              edge_size);
        edge_size++;
      }
    }

    sample_occupation_.clear();
    th_edges_.clear();
    checked_edges_.clear();
  }

  void DRMSpace::buildGraphThreadFn(int thread_id,
      std::pair<unsigned int, unsigned int> &volumes, double dmax)
  {
    th_edges_[thread_id].clear();
    //For each volume
    for (unsigned v = volumes.first; v < volumes.second; v++)
    {
//    if (thread_id == 3)
//      ROS_INFO_STREAM("Thread 3 ["<<volumes.first<<"-"<<volumes.second<<"]: "<<v);
      //Find neighbour volumes
      std::vector<std::pair<unsigned int, double> > neighbours =
          getNeighborIndices(v, 1);
      //For each sample in the central volume
      for (unsigned long int s = 0; s < volumes_[v].reach_samples.size(); s++)
      {
        unsigned long int index = volumes_[v].reach_samples[s];
        //For each neighbour
        for (unsigned int n = 0; n < neighbours.size(); n++)
        {
          //For each sample in the neighbour volume
          for (unsigned long int sn = 0;
              sn < volumes_[neighbours[n].first].reach_samples.size(); sn++)
          {
            unsigned long int n_index =
                volumes_[neighbours[n].first].reach_samples[sn];
            //Check if the edge already exists
            if (newEdge(index, n_index))
            {
              double dist =
                  (Eigen::Map<Eigen::VectorXf>(samples_[index].q, dimension_)
                      - Eigen::Map<Eigen::VectorXf>(samples_[n_index].q,
                          dimension_)).norm();    //.cwiseAbs().maxCoeff();
              //Check connect distance threshold
              if (dist < sqrt(dimension_) * dmax)
              {
                addEdge(index, n_index);
                std::map<unsigned int, bool> edge_volume_occup;
                for (unsigned int vo = 0; vo < sample_occupation_[index].size();
                    vo++)
                  edge_volume_occup[sample_occupation_[index][vo]] = true;
                for (unsigned int vo = 0;
                    vo < sample_occupation_[n_index].size(); vo++)
                  if (edge_volume_occup.find(sample_occupation_[n_index][vo])
                      == edge_volume_occup.end())
                    edge_volume_occup[sample_occupation_[n_index][vo]] = true;
                std::pair<Edge, std::vector<unsigned int> > tmp_edge_occup;
                tmp_edge_occup.first = Edge(index, n_index, dist);
                for (auto &it : edge_volume_occup)
                  tmp_edge_occup.second.push_back(it.first);
                th_edges_[thread_id].push_back(tmp_edge_occup);
              }
            }
          }
        }
      }
    }
    ROS_INFO_STREAM(
        "Thread "<<thread_id<<" finished with "<<th_edges_[thread_id].size()<<" edges, dmax="<<dmax);
  }

  bool DRMSpace::newEdge(unsigned long int a, unsigned long int b)
  {
    if (checked_edges_.find(std::min(a, b)) == checked_edges_.end())
      return true;
    else if (checked_edges_.find(std::min(a, b))->second == std::max(a, b))
      return false;
    else
      return true;
  }

  void DRMSpace::addEdge(unsigned long int a, unsigned long int b)
  {
    boost::mutex::scoped_lock(check_edges_lock_);
    checked_edges_[std::min(a, b)] = std::max(a, b);
  }
}


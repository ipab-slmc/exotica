/*
 * DRMSpace.h
 *
 *  Created on: 16 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACE_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/function/function0.hpp>
#include <boost/thread/thread.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "dynamic_reachability_map/Conversions.h"

namespace dynamic_reachability_map
{
  struct SpaceBounds
  {
      SpaceBounds();
      SpaceBounds(double xlow, double xup, double ylow, double yup, double zlow,
          double zup);

      bool isValid();

      bool inBounds(const geometry_msgs::Point &p);
      bool inBounds(const Eigen::Affine3d &p);

      void print();

      double x_low;
      double x_upper;
      double y_low;
      double y_upper;
      double z_low;
      double z_upper;
  };

  struct VolumeBounds
  {
      VolumeBounds();

      VolumeBounds(const SpaceBounds &bounds, double volume_resolution);

      bool inBounds(const std::vector<int> &p);

      int bx;
      int by;
      int bz;
  };

  struct SparseBitSets
  {
      struct SparsePair
      {
          unsigned int low;
          unsigned int upper;
      };
      SparseBitSets();

      ~SparseBitSets();

      void setFromBoolVector(const std::vector<bool> &bitset);

      std::vector<int> getOccupList();

      unsigned int pair_size;
      SparsePair* data;
  };

  struct Node
  {
      Node();

      ~Node();

      void invalidate();

      float* q;
      geometry_msgs::Pose effpose;
      unsigned int eff_index;
      bool isValid;
      std::vector<unsigned long int> edges;
      Eigen::VectorXf drake_q;
  };

  struct Volume
  {
      Volume();
      ~Volume();
      geometry_msgs::Point center;
      std::vector<unsigned long int> occup_samples;
      std::vector<unsigned long int> reach_samples;
      std::vector<std::vector<unsigned long int> > reach_clusters;
      std::vector<unsigned long int> occup_edges;
      bool isFree;
  };

  struct Edge
  {
      Edge();
      Edge(unsigned long int a_, unsigned long int b_, double length_);
      ~Edge();
      unsigned long int a;
      unsigned long int b;
      double length;
      bool isValid;
  };

  class DRMSpace
  {
      friend class DRMSpaceSaver;
      friend class DRMSpaceLoader;
      friend class MultiThreadsSpaceOccupationLoader;
      friend class DRMSampler;
      friend class DRMFullBodySampler;
      friend class DRMSampleCluster;
      friend class DRM;
    public:
      DRMSpace();
      ~DRMSpace();

      bool createSpace(const SpaceBounds &bounds, double volume_resolution,
          const robot_model::RobotModelConstPtr &model, const std::string & eff,
          const std::string & group_name);

      void clear();
      const SpaceBounds & getSpaceBounds();

      const VolumeBounds & getVolumeBounds();

      bool isReachable(const geometry_msgs::Point &p);

      double getResolution();

      unsigned int getSpaceSize();

      unsigned long int getSampleSize();

      int getDimension();

      bool getVolumeIndex(const geometry_msgs::Point &p, unsigned int & index);
      bool getVolumeIndex(const Eigen::Affine3d &p, unsigned int & index);
      std::vector<std::pair<unsigned int, double> > getNeighborIndices(
          unsigned int index, unsigned int depth);
      double getDistance(unsigned int a, unsigned int b);

      void registOccupation(const Node* const &node);
      void initialiseSamples(int sample_size);
      void reserveSamples(int sample_size);

      const Volume & at(unsigned x, unsigned y, unsigned z) const;

      const Volume & at(unsigned index) const;

      Volume & atNonConst(unsigned x, unsigned y, unsigned z);

      Volume & atNonConst(unsigned int index);

      void addOccupSample(unsigned int volume_index,
          unsigned long int sample_index);
      void addReachSample(unsigned int volume_index,
          unsigned long int sample_index);

      Node & getSampleNonConst(unsigned long int index);

      const Node & getSample(unsigned int index) const;

      void setVolumeOccupied(unsigned int index,
          bool invalidate_samples = true);
      void setVolumeFree(unsigned int index, bool free_samples = true);

      std::vector<unsigned int> getVolumeReachabilities();
      unsigned long int CurrentlyReachability(unsigned int index,
          std::vector<unsigned long int> & valid_samples);
      unsigned long int CurrentlyReachability(unsigned int index,
          const std::vector<unsigned int> &invalid_clusters,
          const std::vector<unsigned int> &invalid_cluster_cells,
          std::vector<unsigned long int> & valid_samples,
          std::vector<std::pair<unsigned int, unsigned int>> & sample_info);
      void qArray2Eigen(const float* q, Eigen::VectorXf &eigen);
      void qEigen2Array(const Eigen::VectorXf &eigen, float* q);
      planning_scene::PlanningScenePtr & getPlanningScene();
      const robot_state::JointModelGroup *getGroup();

      void buildConnectionGraph(double dmax);
      std::vector<Edge> edges_;
    private:
      SpaceBounds space_bounds_;
      VolumeBounds volume_bounds_;
      double resolution_;
      unsigned int space_size_;
      std::vector<Volume> volumes_;
      std::vector<Node> samples_;
      unsigned long int sample_size_;
      planning_scene::PlanningScenePtr ps_;
      std::string eff_;
      const robot_state::JointModelGroup *group_;
      std::vector<int> var_index_;
      int dimension_;
      int thread_size_;
      boost::mutex space_lock_;
      boost::ptr_vector<boost::mutex> volume_locks_;

      void buildGraphThreadFn(int thread_id,
          std::pair<unsigned int, unsigned int> &volumes, double dmax);
      bool newEdge(unsigned long int a, unsigned long int b);
      void addEdge(unsigned long int a, unsigned long int b);
      std::vector<std::vector<std::pair<Edge, std::vector<unsigned int> > > > th_edges_;
      std::vector<std::vector<unsigned int> > sample_occupation_;
      std::map<unsigned long int, unsigned long int> checked_edges_;
      boost::mutex check_edges_lock_;
  };
  typedef boost::shared_ptr<DRMSpace> DRMSpace_ptr;
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACE_H_ */

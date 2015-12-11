/*
 * DRMSpaceLoader.cpp
 *
 *  Created on: 21 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMSpaceLoader.h"

namespace dynamic_reachability_map
{

  DRMSpaceLoader::DRMSpaceLoader()
  {

  }

  DRMSpaceLoader::~DRMSpaceLoader()
  {

  }

  bool DRMSpaceLoader::loadSpace(const std::string &path, DRMSpace_ptr &space,
      const robot_model::RobotModelConstPtr &model)
  {
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(path.c_str())) == NULL)
    {
      ROS_ERROR_STREAM("Result directory "<<path<<" does not exist");
      return false;
    }
    std::map<std::string, bool> files;
    files["samples.bin"] = false;
    files["reach.txt"] = false;
    files["info.xml"] = false;
    files["drake_configurations.bin"] = false;
    while ((dirp = readdir(dp)) != NULL)
    {
      if (files.find(dirp->d_name) != files.end()) files.at(dirp->d_name) =
          true;
    }
    for (auto &it : files)
      if (!it.second)
      {
        ROS_ERROR_STREAM("File "<<it.first<<" does not exist");
        return false;
      }
    tinyxml2::XMLDocument xml_file;
    xml_file.Clear();

    std::string info_path = path + "/info.xml";
    if (xml_file.LoadFile(info_path.c_str()) != tinyxml2::XML_NO_ERROR)
    {
      xml_file.Clear();
      ROS_ERROR_STREAM("Bad XML information file "<<info_path);
      return false;
    }
    tinyxml2::XMLHandle drm_handle(
        xml_file.FirstChildElement("DynamicReachabilityMap"));
    if (!drm_handle.ToElement())
    {
      ROS_ERROR("XML file does not have DynamicReachabilityMap tag");
      return false;
    }

    tinyxml2::XMLHandle tmp_handle = drm_handle.FirstChildElement("Robot");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR("DynamicReachabilityMap does not have Robot tag");
      return false;
    }

    std::string robot_name = tmp_handle.ToElement()->Attribute("name");
    if (robot_name.compare(model->getName()) != 0)
    {
      ROS_ERROR_STREAM(
          "Robot model has name "<<model->getName()<<", but DRM is created with "<<robot_name);
      return false;
    }

    tmp_handle = drm_handle.FirstChildElement("SpaceBounds");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR("DynamicReachabilityMap does not have SpaceBounds tag");
      return false;
    }

    SpaceBounds bounds;
    tinyxml2::XMLHandle bounds_handle = tmp_handle.FirstChildElement("XLower");
    if (!bounds_handle.ToElement())
    {
      ROS_ERROR("SpaceBounds does not have XLower tag");
      return false;
    }
    else
    {
      bounds.x_low = std::stod(bounds_handle.ToElement()->Attribute("value"));
    }

    bounds_handle = tmp_handle.FirstChildElement("XUpper");
    if (!bounds_handle.ToElement())
    {
      ROS_ERROR("SpaceBounds does not have XUpper tag");
      return false;
    }
    else
    {
      bounds.x_upper = std::stod(bounds_handle.ToElement()->Attribute("value"));
    }

    bounds_handle = tmp_handle.FirstChildElement("YLower");
    if (!bounds_handle.ToElement())
    {
      ROS_ERROR("SpaceBounds does not have YLower tag");
      return false;
    }
    else
    {
      bounds.y_low = std::stod(bounds_handle.ToElement()->Attribute("value"));
    }

    bounds_handle = tmp_handle.FirstChildElement("YUpper");
    if (!bounds_handle.ToElement())
    {
      ROS_ERROR("SpaceBounds does not have YUpper tag");
      return false;
    }
    else
    {
      bounds.y_upper = std::stod(bounds_handle.ToElement()->Attribute("value"));
    }

    bounds_handle = tmp_handle.FirstChildElement("ZLower");
    if (!bounds_handle.ToElement())
    {
      ROS_ERROR("SpaceBounds does not have ZLower tag");
      return false;
    }
    else
    {
      bounds.z_low = std::stod(bounds_handle.ToElement()->Attribute("value"));
    }

    bounds_handle = tmp_handle.FirstChildElement("ZUpper");
    if (!bounds_handle.ToElement())
    {
      ROS_ERROR("SpaceBounds does not have ZUpper tag");
      return false;
    }
    else
    {
      bounds.z_upper = std::stod(bounds_handle.ToElement()->Attribute("value"));
    }

    tmp_handle = drm_handle.FirstChildElement("VolumeResolution");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR("DynamicReachabilityMap does not have VolumeResolution tag");
      return false;
    }
    double resolution = std::stod(tmp_handle.ToElement()->Attribute("value"));

    tinyxml2::XMLHandle group_handle = drm_handle.FirstChildElement(
        "PlanningGroup");
    if (!group_handle.ToElement())
    {
      ROS_ERROR("DynamicReachabilityMap does not have PlanningGroup tag");
      return false;
    }

    std::string group_name = group_handle.ToElement()->Attribute("name");
    if (!model->hasJointModelGroup(group_name))
    {
      ROS_ERROR_STREAM("Robot model does not contain group "<<group_name);
      return false;
    }

    tmp_handle = group_handle.FirstChildElement("EndEffector");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR("PlanningGroup does not have EndEffector tag");
      return false;
    }

    std::string eff = tmp_handle.ToElement()->Attribute("name");
    if (!model->hasLinkModel(eff))
    {
      ROS_ERROR_STREAM(
          "Robot "<<model->getName()<<" does not have end-effector link "<<eff);
      return false;
    }

    if (!space->createSpace(bounds, resolution, model, eff, group_name))
    {
      ROS_ERROR("Can not create DRM space");
      return false;
    }

    tmp_handle = drm_handle.FirstChildElement("SpaceSize");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR("DynamicReachabilityMap does not have SpaceSize tag");
      return false;
    }
    int tmp_space_size = std::stoi(tmp_handle.ToElement()->Attribute("value"));
    if (space->getSpaceSize() != tmp_space_size)
    {
      ROS_ERROR_STREAM(
          "Recorded space size ("<<tmp_space_size<<") and actual space size ("<<space->getSpaceSize()<<") does not match !");
      return false;
    }

    tmp_handle = drm_handle.FirstChildElement("SampleSize");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR("DynamicReachabilityMap does not have SampleSize tag");
      return false;
    }

    int sample_size = std::stoi(tmp_handle.ToElement()->Attribute("value"));
    space->initialiseSamples(sample_size);

    tmp_handle = drm_handle.FirstChildElement("OccupationSavingThreads");
    if (!tmp_handle.ToElement())
    {
      ROS_ERROR(
          "DynamicReachabilityMap does not have OccupationSavingThreads tag");
      return false;
    }

    int thread_size = std::stoi(tmp_handle.ToElement()->Attribute("value"));
    if (thread_size <= 0 || thread_size > 8)
    {
      ROS_ERROR_STREAM("Invalid thread size "<<thread_size);
      return false;
    }
    space->thread_size_ = thread_size;

    std::map<std::string, bool> occup_files;
    for (int t = 0; t < thread_size; t++)
      occup_files["space_occupation_" + std::to_string(t) + ".bin"] = false;
    DIR *occup_dp;
    if ((occup_dp = opendir((path + "/space_occupation").c_str())) == NULL)
    {
      ROS_ERROR_STREAM(
          "Result directory "<<path<<"/space_occupation does not exist");
      return false;
    }
    while ((dirp = readdir(occup_dp)) != NULL)
    {
      if (occup_files.find(dirp->d_name) != occup_files.end())
        occup_files.at(dirp->d_name) = true;
    }
    for (auto &it : occup_files)
      if (!it.second)
      {
        ROS_ERROR_STREAM("File "<<it.first<<" does not exist");
        return false;
      }

    std::string line;
    //Loading samples
    ROS_INFO("Loading samples");
    std::ifstream samples_file(path + "/samples.bin", std::ios_base::binary);
    for (int i = 0; i < sample_size; i++)
    {
      space->samples_[i].isValid = true;

      samples_file.read((char *) &space->samples_[i].eff_index,
          sizeof(unsigned int));
      samples_file.read((char *) &space->samples_[i].effpose.position.x,
          sizeof(float));
      samples_file.read((char *) &space->samples_[i].effpose.position.y,
          sizeof(float));
      samples_file.read((char *) &space->samples_[i].effpose.position.z,
          sizeof(float));
      samples_file.read((char *) &space->samples_[i].effpose.orientation.x,
          sizeof(float));
      samples_file.read((char *) &space->samples_[i].effpose.orientation.y,
          sizeof(float));
      samples_file.read((char *) &space->samples_[i].effpose.orientation.z,
          sizeof(float));
      samples_file.read((char *) &space->samples_[i].effpose.orientation.w,
          sizeof(float));
      for (int j = 0; j < space->dimension_; j++)
        samples_file.read((char *) &space->samples_[i].q[j], sizeof(float));
    }
    samples_file.close();

    ROS_INFO("Loading drake configurations");
    std::ifstream drake_file(path + "/drake_configurations.bin",
        std::ios_base::binary);
    for (int i = 0; i < sample_size; i++)
    {
      space->samples_[i].drake_q.resize(38);
      for (int j = 0; j < 38; j++)
        drake_file.read((char *) &space->samples_[i].drake_q(j),
            sizeof(float));
    }
    drake_file.close();

    //Loading reach samples
    ROS_INFO("Loading reach samples");
    std::ifstream reach_file;
    reach_file.open(path + "/reach.txt");
    for (int i = 0; i < space->space_size_; i++)
    {
      getline(reach_file, line);
      std::vector<std::string> reaches = getStringVector(line);
      for (int j = 0; j < reaches.size(); j++)
        space->volumes_[i].reach_samples.push_back(
            (unsigned long int) std::stoi(reaches[j]));
    }

    //Loading occupation samples
    ROS_INFO("Loading occupation samples");
    boost::shared_ptr<MultiThreadsSpaceOccupationLoader> mtl;
    mtl.reset(
        new MultiThreadsSpaceOccupationLoader(path + "/space_occupation",
            space));
    mtl->loadSpaceOccupation();
    return true;
  }

  DRMSpaceLoader::MultiThreadsSpaceOccupationLoader::MultiThreadsSpaceOccupationLoader(
      const std::string & path, DRMSpace_ptr &space)
      : path_(path), space_(space)
  {

  }

  DRMSpaceLoader::MultiThreadsSpaceOccupationLoader::~MultiThreadsSpaceOccupationLoader()
  {

  }

  void DRMSpaceLoader::MultiThreadsSpaceOccupationLoader::loadSpaceOccupation()
  {
    std::vector<boost::thread*> th(space_->thread_size_);
    for (unsigned int i = 0; i < space_->thread_size_; ++i)
      th[i] = new boost::thread(
          boost::bind(&MultiThreadsSpaceOccupationLoader::load, this, i));
    for (unsigned int i = 0; i < space_->thread_size_; ++i)
    {
      th[i]->join();
      delete th[i];
    }
  }

  void DRMSpaceLoader::MultiThreadsSpaceOccupationLoader::load(int id)
  {
    ROS_INFO_STREAM("MultiThreadsSpaceOccupationLoader "<<id<<" start");
    std::ifstream space_occupied_file(
        path_ + "/space_occupation_" + std::to_string(id) + ".bin",
        std::ios_base::binary);
    unsigned int index = 0;
    unsigned long int occup_size = 0;
    while (!space_occupied_file.eof())
    {
      space_occupied_file.read((char *) &index, sizeof(unsigned int));
      space_occupied_file.read((char *) &occup_size, sizeof(unsigned long int));
      if (occup_size == 2)
      {
        unsigned long int i0 = 0, i1 = 0;
        space_occupied_file.read((char *) &i0, sizeof(unsigned long int));
        space_occupied_file.read((char *) &i1, sizeof(unsigned long int));
        if (i0 == 0 && i1 == 0)
        {
          space_->volumes_[index].occup_samples.resize(space_->sample_size_);
          for (unsigned long int i = 0; i < space_->sample_size_; i++)
            space_->volumes_[index].occup_samples[i] = i;
        }
        else
        {
          space_->volumes_[index].occup_samples.resize(2);
          space_->volumes_[index].occup_samples[0] = i0;
          space_->volumes_[index].occup_samples[1] = i1;
        }
      }
      else
      {
        space_->volumes_[index].occup_samples.resize(occup_size);
        space_occupied_file.read(
            (char *) &space_->volumes_[index].occup_samples[0],
            occup_size * sizeof(unsigned long int));
      }
//    if ()
//    {
//      unsigned int index = (unsigned int)std::stoi(data[0]);
//      if (0 > index || index >= space_->space_size_)
//      {
//        ROS_ERROR("MultiThreadsSpaceOccupationLoader found an invalid volume index, this should never happen !");
//      }
//      else
//      {
//        if (data.size() == 3 && data[1] == "1" && data[2] == "1")
//        {
//          space_->volumes_[index].occup_samples.resize(space_->sample_size_);
//          for (unsigned long int i = 0; i < space_->sample_size_; i++)
//            space_->volumes_[index].occup_samples[i] = i;
//        }
//        else
//        {
//          space_->volumes_[index].occup_samples.resize(data.size() - 1);
//          for (int i = 0; i < data.size() - 1; i++)
//            space_->volumes_[index].occup_samples[i] = (unsigned long int)std::stoi(data[i + 1]);
//        }
//      }
//    }
    }
  }
}


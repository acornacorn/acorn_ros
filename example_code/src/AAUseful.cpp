CONTENTS
poseString                 - print Affine3d as 4x4 matrix
posString                  - print vector
matrixTFToEigen            - various conversions
markDebugPosition          - add an rviz marker at a position.
findFile                   - look up dir tree to find a file


#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

static std::string poseString(const Eigen::Affine3d& pose, const std::string& pfx = "")
{
  std::stringstream ss;
  ss.precision(3);
  for (int y=0;y<4;y++)
  {
    ss << pfx;
    for (int x=0;x<4;x++)
    {
      ss << std::setw(8) << pose(y,x) << " ";
    }
    ss << std::endl;
  }
  return ss.str();
}

static std::string poseString(const geometry_msgs::Pose& pose, const std::string& pfx = "")
{
  Eigen::Affine3d epose;
  tf::poseMsgToEigen(pose, epose);
  return poseString(epose, pfx);
}

static std::string posString(const Eigen::Vector3d& pos, const std::string& pfx = "(", const std::string& sfx = ")")
{
  std::stringstream ss;
  ss.precision(3);
  ss << pfx;
  for (int x=0;x<3;x++)
  {
    if (x)
      ss << ", ";
    ss << std::setw(8) << pos(x);
  }
  ss << sfx;
  return ss.str();
}


#include <tf_conversions/tf_eigen.h>
void tf::matrixTFToEigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e);
void tf::matrixEigenToTF(const Eigen::Matrix3d &e, tf::Matrix3x3 &t);
void tf::poseTFToEigen(const tf::Pose &t, Eigen::Affine3d &e);
void tf::poseEigenToTF(const Eigen::Affine3d &e, tf::Pose &t);
void tf::quaternionTFToEigen(const tf::Quaternion &t, Eigen::Quaterniond &e);
void tf::quaternionEigenToTF(const Eigen::Quaterniond &e, tf::Quaternion &t);
void tf::transformTFToEigen(const tf::Transform &t, Eigen::Affine3d &e);
void tf::transformEigenToTF(const Eigen::Affine3d &e, tf::Transform &t);
void tf::vectorTFToEigen(const tf::Vector3 &t, Eigen::Vector3d &e);
void tf::vectorEigenToTF(const Eigen::Vector3d &e, tf::Vector3 &t);


  tf::StampedTransform xform;
  tf_listener_.lookupTransform("from_frame"
                               "to_frame",
                                ros::Time(),
                                xform);
  Eigen::Affine3d pose;
  tf::transformTFToEigen(xform, pose);


  

static boost::shared_ptr<rviz::MarkerBase> g_mark[20];
static Ogre::SceneNode *g_mark_node = NULL;
void AtlasDisplay::markDebugPosition(int idx, const Eigen::Vector3d& pos, const Eigen::Vector4f& color, double radius)
{
  if (idx<0 || idx>=sizeof(g_mark)/sizeof(g_mark[0]))
    return;
  boost::shared_ptr<rviz::MarkerBase>& mptr = g_mark[idx];

  if (!g_mark_node)
    g_mark_node = scene_node_->createChildSceneNode();

  rviz::MarkerBase *m;
#if 0
  mptr.reset(new rviz::ShapeMarker(0, context_, g_mark_node));
  m = mptr.get();
#else
  m = new rviz::ShapeMarker(0, context_, g_mark_node);
#endif

  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = radius * 0.5;
  sphere_marker.scale.y = radius * 0.5;
  sphere_marker.scale.z = radius * 0.5;
  sphere_marker.color.r = color.x();
  sphere_marker.color.g = color.y();
  sphere_marker.color.b = color.z();
  sphere_marker.color.a = color.w();
  sphere_marker.header.frame_id = context_->getFrameManager()->getFixedFrame();

  m->setMessage(sphere_marker);
  Ogre::Vector3 opos(pos.x(), pos.y(), pos.z());
  m->setPosition(opos);
  m->setOrientation(Ogre::Quaternion(1,0,0,0));

  //m->setInteractiveObject(shared_from_this());
}


//###########################################################################
//############################### FILESYSTEM ################################
//###########################################################################
look in PWD, parent, parent, ... to find a file.
find_package(Boost REQUIRED system filesystem)
#include <boost/filesystem.hpp>
static boost::filesystem::path findFile(const boost::filesystem::path& relpath)
{
  if (relpath.is_absolute())
    return relpath;
  boost::filesystem::path base = boost::filesystem::current_path();
  while (!base.empty())
  {
    if (boost::filesystem::exists(base / relpath))
      return base / relpath;

    base = base.parent_path();
  }
  return relpath;
}

// example:  
//    shapes::createMeshFromResource(
//                "file://" +
//                findFile("src/geometric_shapes/test/resources/forearm_roll.stl").string());



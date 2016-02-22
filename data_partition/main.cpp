/** @author Kanzhi Wu
 *  @date   29/01/2016
 *
 */



#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/io/ply_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/common/transforms.h>
//#include <pcl/common/impl/transforms.hpp>
#include <pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/impl/octree2buf_base.hpp>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_search.hpp>
#include <pcl/octree/impl/octree_iterator.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>
#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/thread.hpp>


#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>


/// struct PointNormalL
struct PointNormalL{
  PCL_ADD_POINT4D
  PCL_ADD_NORMAL4D
  int label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointNormalL,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (int, label, label)
)





//=============================
// Displaying cubes is very long!
// so we limit their numbers.
 const int MAX_DISPLAYED_CUBES(15000);


/// octree viewer

 class OctreeViewer
 {
 public:
   OctreeViewer (std::string &filename, double resolution) :
     viz ("Octree visualizator"), cloud (new pcl::PointCloud<PointNormalL>()),
         displayCloud (new pcl::PointCloud<PointNormalL>()), octree (resolution), displayCubes(false),
         showPointsWithCubes (false), wireframe (true)
   {

     //try to load the cloud
     if (!loadCloud(filename))
       return;

     //register keyboard callbacks
     viz.registerKeyboardCallback(&OctreeViewer::keyboardEventOccurred, *this, 0);

     //key legends
     viz.addText("Keys:", 0, 170, 0.0, 1.0, 0.0, "keys_t");
     viz.addText("a -> Increment displayed depth", 10, 155, 0.0, 1.0, 0.0, "key_a_t");
     viz.addText("z -> Decrement displayed depth", 10, 140, 0.0, 1.0, 0.0, "key_z_t");
     viz.addText("d -> Toggle Point/Cube representation", 10, 125, 0.0, 1.0, 0.0, "key_d_t");
     viz.addText("x -> Show/Hide original cloud", 10, 110, 0.0, 1.0, 0.0, "key_x_t");
     viz.addText("s/w -> Surface/Wireframe representation", 10, 95, 0.0, 1.0, 0.0, "key_sw_t");

     //set current level to half the maximum one
     displayedDepth = static_cast<int> (floor (octree.getTreeDepth() / 2.0));
     if (displayedDepth == 0)
       displayedDepth = 1;

     //show octree at default depth
     extractPointsAtLevel(displayedDepth);

     //reset camera
     viz.resetCameraViewpoint("cloud");

     //run main loop
     run();

   }


   OctreeViewer (pcl::PointCloud<PointNormalL>::Ptr lcloud, double resolution) :
     viz ("Octree visualizator"), cloud (new pcl::PointCloud<PointNormalL>()),
         displayCloud (new pcl::PointCloud<PointNormalL>()), octree (resolution), displayCubes(false),
         showPointsWithCubes (false), wireframe (true)
   {

     //try to load the cloud
     if (!loadCloud(lcloud))
       return;

     //register keyboard callbacks
     viz.registerKeyboardCallback(&OctreeViewer::keyboardEventOccurred, *this, 0);

     //key legends
     viz.addText("Keys:", 0, 170, 0.0, 1.0, 0.0, "keys_t");
     viz.addText("a -> Increment displayed depth", 10, 155, 0.0, 1.0, 0.0, "key_a_t");
     viz.addText("z -> Decrement displayed depth", 10, 140, 0.0, 1.0, 0.0, "key_z_t");
     viz.addText("d -> Toggle Point/Cube representation", 10, 125, 0.0, 1.0, 0.0, "key_d_t");
     viz.addText("x -> Show/Hide original cloud", 10, 110, 0.0, 1.0, 0.0, "key_x_t");
     viz.addText("s/w -> Surface/Wireframe representation", 10, 95, 0.0, 1.0, 0.0, "key_sw_t");

     //set current level to half the maximum one
     displayedDepth = static_cast<int> (floor (octree.getTreeDepth() / 2.0));
     if (displayedDepth == 0)
       displayedDepth = 1;

     //show octree at default depth
     extractPointsAtLevel(displayedDepth);

     //reset camera
     viz.resetCameraViewpoint("cloud");

     //run main loop
     run();

   }

 private:
   //========================================================
   // PRIVATE ATTRIBUTES
   //========================================================
   //visualizer
   pcl::PointCloud<PointNormalL>::Ptr xyz;


   pcl::visualization::PCLVisualizer viz;
   //original cloud
   pcl::PointCloud<PointNormalL>::Ptr cloud;
   //displayed_cloud
   pcl::PointCloud<PointNormalL>::Ptr displayCloud;
   //octree
   pcl::octree::OctreePointCloudVoxelCentroid<PointNormalL> octree;
   //level
   int displayedDepth;
   //bool to decide if we display points or cubes
   bool displayCubes, showPointsWithCubes, wireframe;
   //========================================================

   /* \brief Callback to interact with the keyboard
    *
    */
   void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
   {

     if (event.getKeySym() == "a" && event.keyDown())
     {
       IncrementLevel();
     }
     else if (event.getKeySym() == "z" && event.keyDown())
     {
       DecrementLevel();
     }
     else if (event.getKeySym() == "d" && event.keyDown())
     {
       displayCubes = !displayCubes;
       update();
     }
     else if (event.getKeySym() == "x" && event.keyDown())
     {
       showPointsWithCubes = !showPointsWithCubes;
       update();
     }
     else if (event.getKeySym() == "w" && event.keyDown())
     {
       if(!wireframe)
         wireframe=true;
       update();
     }
     else if (event.getKeySym() == "s" && event.keyDown())
     {
       if(wireframe)
         wireframe=false;
       update();
     }
   }

   /* \brief Graphic loop for the viewer
    *
    */
   void run()
   {
     while (!viz.wasStopped())
     {
       //main loop of the visualizer
       viz.spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
     }
   }


   /* \brief loadcloud from point cloud xyzl
    * also initilize the octree
    */
   bool loadCloud(pcl::PointCloud<PointNormalL>::Ptr lcloud)
   {
     if ( lcloud->empty() )
       return false;

     // convert lcloud to cloud
     for ( int i = 0; i < lcloud->points.size(); ++ i ) {
       PointNormalL pt;
       pt.x = lcloud->points[i].x;
       pt.y = lcloud->points[i].y;
       pt.z = lcloud->points[i].z;
       cloud->points.push_back( pt );
     }
     cloud->width = cloud->points.size();
     cloud->height = 1;
     cloud->is_dense = false;

     //remove NaN Points
     std::vector<int> nanIndexes;
 //    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
     std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

     //create octree structure
     octree.setInputCloud(cloud);
     //update bounding box automatically
     octree.defineBoundingBox();
     //add points in the tree
     octree.addPointsFromInputCloud();
     return true;
   }

   /* \brief Helper function that read a pointcloud file (returns false if pbl)
    *  Also initialize the octree
    *
    */
   bool loadCloud(std::string &filename)
   {
     std::cout << "Loading file " << filename.c_str() << std::endl;
     //read cloud
     if (pcl::io::loadPCDFile(filename, *cloud))
     {
       std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
       return false;
     }

     //remove NaN Points
     std::vector<int> nanIndexes;
     pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
     std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

     //create octree structure
     octree.setInputCloud(cloud);
     //update bounding box automatically
     octree.defineBoundingBox();
     //add points in the tree
     octree.addPointsFromInputCloud();
     return true;
   }

   /* \brief Helper function that draw info for the user on the viewer
    *
    */
   void showLegend(bool showCubes)
   {
     char dataDisplay[256];
     sprintf(dataDisplay, "Displaying data as %s", (showCubes) ? ("CUBES") : ("POINTS"));
     viz.removeShape("disp_t");
     viz.addText(dataDisplay, 0, 60, 1.0, 0.0, 0.0, "disp_t");

     char level[256];
     sprintf(level, "Displayed depth is %d on %d", displayedDepth, octree.getTreeDepth());
     viz.removeShape("level_t1");
     viz.addText(level, 0, 45, 1.0, 0.0, 0.0, "level_t1");

     viz.removeShape("level_t2");
     sprintf(level, "Voxel size: %.4fm [%lu voxels]", sqrt(octree.getVoxelSquaredSideLen(displayedDepth)),
             displayCloud->points.size());
     viz.addText(level, 0, 30, 1.0, 0.0, 0.0, "level_t2");

     viz.removeShape("org_t");
     if (showPointsWithCubes)
       viz.addText("Displaying original cloud", 0, 15, 1.0, 0.0, 0.0, "org_t");
   }

   /* \brief Visual update. Create visualizations and add them to the viewer
    *
    */
   void update()
   {
     //remove existing shapes from visualizer
     clearView();

     //prevent the display of too many cubes
     bool displayCubeLegend = displayCubes && static_cast<int> (displayCloud->points.size ()) <= MAX_DISPLAYED_CUBES;

     showLegend(displayCubeLegend);

     if (displayCubeLegend)
     {
       //show octree as cubes
       showCubes(sqrt(octree.getVoxelSquaredSideLen(displayedDepth)));
       if (showPointsWithCubes)
       {
         //add original cloud in visualizer
         pcl::visualization::PointCloudColorHandlerGenericField<PointNormalL> color_handler(cloud, "z");
         viz.addPointCloud(cloud, color_handler, "cloud");
       }
     }
     else
     {
       //add current cloud in visualizer
       pcl::visualization::PointCloudColorHandlerGenericField<PointNormalL> color_handler(displayCloud,"z");
       viz.addPointCloud(displayCloud, color_handler, "cloud");
     }
   }

   /* \brief remove dynamic objects from the viewer
    *
    */
   void clearView()
   {
     //remove cubes if any
     vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();
     while (renderer->GetActors()->GetNumberOfItems() > 0)
       renderer->RemoveActor(renderer->GetActors()->GetLastActor());
     //remove point clouds if any
     viz.removePointCloud("cloud");
   }

   /* \brief Create a vtkSmartPointer object containing a cube
    *
    */
   vtkSmartPointer<vtkPolyData> GetCuboid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
   {
     vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
     cube->SetBounds(minX, maxX, minY, maxY, minZ, maxZ);
     return cube->GetOutput();
   }

   /* \brief display octree cubes via vtk-functions
    *
    */
   void showCubes(double voxelSideLen)
   {
     //get the renderer of the visualizer object
     vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();

     vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New();
     size_t i;
     double s = voxelSideLen / 2.0;
     for (i = 0; i < displayCloud->points.size(); i++)
     {

       double x = displayCloud->points[i].x;
       double y = displayCloud->points[i].y;
       double z = displayCloud->points[i].z;

 #if VTK_MAJOR_VERSION < 6
       treeWireframe->AddInput(GetCuboid(x - s, x + s, y - s, y + s, z - s, z + s));
 #else
       treeWireframe->AddInputData (GetCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
 #endif
     }

     vtkSmartPointer<vtkActor> treeActor = vtkSmartPointer<vtkActor>::New();

     vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
 #if VTK_MAJOR_VERSION < 6
     mapper->SetInput(treeWireframe->GetOutput());
 #else
     mapper->SetInputData (treeWireframe->GetOutput ());
 #endif
     treeActor->SetMapper(mapper);

     treeActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
     treeActor->GetProperty()->SetLineWidth(2);
     if(wireframe)
     {
       treeActor->GetProperty()->SetRepresentationToWireframe();
       treeActor->GetProperty()->SetOpacity(0.35);
     }
     else
       treeActor->GetProperty()->SetRepresentationToSurface();

     renderer->AddActor(treeActor);
   }

   /* \brief Extracts all the points of depth = level from the octree
    *
    */
   void extractPointsAtLevel(int depth)
   {
     displayCloud->points.clear();

     pcl::octree::OctreePointCloudVoxelCentroid<PointNormalL>::Iterator tree_it;
     pcl::octree::OctreePointCloudVoxelCentroid<PointNormalL>::Iterator tree_it_end = octree.end();

     PointNormalL pt;
     std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
     double start = pcl::getTime ();

     for (tree_it = octree.begin(depth); tree_it!=tree_it_end; ++tree_it)
     {
       Eigen::Vector3f voxel_min, voxel_max;
       octree.getVoxelBounds(tree_it, voxel_min, voxel_max);

       pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
       pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
       pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
       displayCloud->points.push_back(pt);
     }

     double end = pcl::getTime ();
     printf("%lu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
            (end - start) / static_cast<double> (displayCloud->points.size ()));

     update();
   }

   /* \brief Helper function to increase the octree display level by one
    *
    */
   bool IncrementLevel()
   {
     if (displayedDepth < static_cast<int> (octree.getTreeDepth ()))
     {
       displayedDepth++;
       extractPointsAtLevel(displayedDepth);
       return true;
     }
     else
       return false;
   }

   /* \brief Helper function to decrease the octree display level by one
    *
    */
   bool DecrementLevel()
   {
     if (displayedDepth > 0)
     {
       displayedDepth--;
       extractPointsAtLevel(displayedDepth);
       return true;
     }
     return false;
   }

 };




/// pose structure
struct Pose{
  Eigen::Quaternionf quat;
  Eigen::Matrix3f rot;
  Eigen::Vector3f trans;
  Eigen::Matrix4f htransform;

  Pose(float tx, float ty, float tz,
       float qx, float qy, float qz, float qw) {
    trans << tx, ty, tz;
    quat = Eigen::Quaternionf(qw, qx, qy, qz);
    rot = quat.toRotationMatrix();
    htransform = Eigen::Matrix4f::Identity();
    htransform.topLeftCorner(3, 3) = rot;
    htransform.topRightCorner(3, 1) = trans;
  }
};





/// inverse homogeous transformation matrix
Eigen::Matrix4f invhtrans(Eigen::Matrix4f trans) {
//  Eigen::Matrix4f invtrans = Eigen::Matrix4f::Identity();
//  invtrans.topLeftCorner(3,3) = trans.topLeftCorner(3,3).inverse();
//  invtrans.topRightCorner(3,1) = -trans.topRightCorner(3,1);
  Eigen::Matrix4f invtrans = trans.inverse();
  return invtrans;
}


/// load point cloud from txt
pcl::PointCloud<pcl::PointXYZ>::Ptr loadcloud( std::string filename ) {
  std::ifstream intxt( filename.c_str() );
  std::string line;
  getline( intxt, line );
  int npoints = boost::lexical_cast<int>(line);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  std::cout << "Point cloud size = " << npoints << std::endl;
  cloud->width = npoints;
  cloud->height = 1;
  cloud->is_dense = false;
  for ( int i = 0; i < npoints; ++ i ) {
    getline(intxt, line);
    std::vector<std::string> segs;
    boost::algorithm::split( segs, line, boost::algorithm::is_any_of(" ") );
    pcl::PointXYZ pt;
    pt.x = boost::lexical_cast<float>(segs[0]);
    pt.y = boost::lexical_cast<float>(segs[1]);
    pt.z = boost::lexical_cast<float>(segs[2]);
    cloud->points.push_back( pt );
  }
  return cloud;
}


/// generate nperm
std::vector<int> nperm(int n) {
  std::vector< std::pair<float, int> > rdata;(2)
  for ( int i = 0; i < n; ++ i ) {
    rdata.push_back( std::make_pair(float(rand()), i) );
  }
  std::sort( rdata.begin(), rdata.end() );
  std::vector<int> permvec;
  for ( int i = 0; i < n; ++ i )
    permvec.push_back( rdata[i].second );
  return permvec;
}


/// split pointnormalL cloud
void split_pointnormall_cloud( pcl::PointCloud<PointNormalL>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr & xyzcloud,
                               pcl::PointCloud<pcl::Normal>::Ptr & normalcloud ) {
  for ( int i = 0; i < cloud->size(); ++ i ) {
    PointNormalL tpt = cloud->points[i];
    pcl::PointXYZ xyzpt; xyzpt.x = tpt.x; xyzpt.y = tpt.y; xyzpt.z = tpt.z;
    pcl::Normal normalpt;
    normalpt.normal_x = tpt.normal_x;
    normalpt.normal_y = tpt.normal_y;
    normalpt.normal_z = tpt.normal_z;
    xyzcloud->points.push_back( xyzpt );
    normalcloud->points.push_back( normalpt );
  }
  xyzcloud->width = xyzcloud->points.size();
  xyzcloud->height = 1;
  xyzcloud->is_dense = false;
  normalcloud->width = normalcloud->points.size();
  normalcloud->height = 1;
  normalcloud->is_dense = false;
}


/// main func
int main( int argc, char ** argv ) {
#if 1

  std::string dir = argv[1];
  const std::string ply_names[]		= {"bun000", "bun090", "bun180", "bun270"};
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > pointclouds;


  // load point cloud
  for ( int i = 0; i < 4; ++ i ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud( new pcl::PointCloud<pcl::PointXYZ>() );
    std::string txt_name = dir+"/"+ply_names[i]+".txt";
    pointcloud = loadcloud( txt_name );
    pointclouds.push_back( pointcloud );
  }

  // align point cloud according to bun.conf
  Pose p_sensor(-0.0172, -0.0936, -0.734, -0.0461723, 0.970603, -0.235889, 0.0124573);
  Pose p1(0, 0, 0, 0, 0, 0, 1);
  Pose p2(0, 0, 0, 0.000335889, -0.708202, 0.000602459, 0.706009);
  Pose p3(0, 0, 0, -0.00215148, 0.999996, -0.0015001, 0.000892527);
  Pose p4(0, 0, 0, 0.000462632, 0.707006, -0.00333301, 0.7072);
  std::vector<Pose> poses;
  poses.push_back( p1 );
  poses.push_back( p2 );
  poses.push_back( p3 );
  poses.push_back( p4 );

  // rotate point clouds and visualise
  pcl::PointCloud<pcl::PointXYZL>::Ptr ltcloud( new pcl::PointCloud<pcl::PointXYZL>() );

  pcl::PointCloud<PointNormalL>::Ptr pnlcloud( new pcl::PointCloud<PointNormalL>() );


  for ( int i = 0; i < poses.size(); ++ i ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud( *pointclouds[i], *tcloud, invhtrans(poses[i].htransform));
    std::string cloudid = "cloud"+boost::lexical_cast<std::string>(i);
    pcl::io::savePLYFileASCII( cloudid+".ply", *tcloud );
    // normal estimation
//    pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal>() );
//    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setInputCloud( tcloud );
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>() );
//    ne.setSearchMethod( tree );
//    ne.setRadiusSearch( 0.01 );
//    ne.compute(*normals);

    // concatenate fields;
//    pcl::PointCloud<pcl::PointNormal>::Ptr pncloud( new pcl::PointCloud<pcl::PointNormal>() );
//    std::cout << tcloud->size() << " = " << normals->size() << std::endl;
//    pcl::concatenateFields( *tcloud, *normals, *pncloud );
//    pcl::io::savePCDFileASCII( "f"+ply_names[i]+".pcd", *pncloud );

    // generate random sample points
//    pcl::PointCloud<pcl::PointNormal>::Ptr fpncloud( new pcl::PointCloud<pcl::PointNormal>() );
//    std::vector<int> permvec = nperm( pncloud->size() );
//    int nfcloud = pncloud->size()*0.1;
//    for ( int ii = 0; ii < nfcloud; ++ ii ) {
//      fpncloud->points.push_back( pncloud->points[permvec[ii]] );
//    }
//    fpncloud->width = fpncloud->points.size();
//    fpncloud->height = 1;
//    fpncloud->is_dense = false;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(tcloud, rand()%255, rand()%255, rand()%255);
//    viewer->addPointCloud<pcl::PointXYZ> (tcloud, color, cloudid);
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudid);
//    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (tcloud, normals, 10, 0.02, "normals"+cloudid);

//    viewer->addCoordinateSystem (0.05);
//    viewer->initCameraParameters ();
//    while (!viewer->wasStopped ()) {
//      viewer->spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

//    // generate rbgl point cloud
//    for ( int j = 0; j < tcloud->points.size(); ++ j ) {
//      PointNormalL pnlpt;
//      pcl::PointXYZ tpt = tcloud->points[j];
//      pcl::Normal npt = normals->points[j];
//      pnlpt.x = tpt.x; pnlpt.y = tpt.y; pnlpt.z = tpt.z;
//      pnlpt.normal_x = npt.normal_x; pnlpt.normal_y = npt.normal_y; pnlpt.normal_z = npt.normal_z;
//      pnlpt.label = i;
//      pnlcloud->points.push_back( pnlpt );
//    }
  }
  pnlcloud->width = pnlcloud->points.size();
  pnlcloud->height = 1;
  pnlcloud->is_dense = false;
  Eigen::Vector3f minvec, maxvec;
  minvec << 1.0, 1.0, 1.0; maxvec << -1.0, -1.0, -1.0;
  for ( int i = 0; i < pnlcloud->size(); ++ i ) {
    PointNormalL tpt = pnlcloud->points[i];
    if ( minvec[0] > tpt.x ) minvec[0] = tpt.x;
    if ( minvec[1] > tpt.y ) minvec[1] = tpt.y;
    if ( minvec[2] > tpt.z ) minvec[2] = tpt.z;
    if ( maxvec[0] < tpt.x ) maxvec[0] = tpt.x;
    if ( maxvec[1] < tpt.y ) maxvec[1] = tpt.y;
    if ( maxvec[2] < tpt.z ) maxvec[2] = tpt.z;
  }
  Eigen::Vector3f centroidvec = (minvec+maxvec)/2;
  centroidvec[2] -= 0.002;
  std::cout << "min point: " << minvec.transpose() << " and max point: " << maxvec.transpose() << " and centroid " << centroidvec.transpose() << std::endl;

  // adjust normal direction
  float len_normal = 0.02f;
//  PointNormalL minpt, maxpt;
//  pcl::getMinMax3D( pnlcloud, minpt, maxpt );
//  Vector3f cntvec = ( minpt.getArray3fMap()+maxpt.getArray3fMap() )/2;
  for ( int i = 0; i < pnlcloud->size(); ++ i ) {
    PointNormalL & tpt = pnlcloud->points[i];
    Eigen::Vector3f invec = tpt.getVector3fMap()-len_normal*tpt.getNormalVector3fMap();
    float indist = (invec-centroidvec).norm();
    Eigen::Vector3f outvec = tpt.getVector3fMap()+len_normal*tpt.getNormalVector3fMap();
    float outdist = (outvec-centroidvec).norm();
    if ( indist > outdist ) {
      tpt.normal_x *= -1;
      tpt.normal_y *= -1;
      tpt.normal_z *= -1;
    }
  }

  pcl::io::savePLYFileASCII( "cloud.ply", *pnlcloud );
#else
  pcl::PointCloud<PointNormalL>::Ptr pnlcloud( new pcl::PointCloud<PointNormalL>() );
  pcl::io::loadPCDFile( "cloud.pcd", *pnlcloud );
  Eigen::Vector3f minvec, maxvec;
  minvec << 1.0, 1.0, 1.0; maxvec << -1.0, -1.0, -1.0;
  for ( int i = 0; i < pnlcloud->size(); ++ i ) {
    PointNormalL tpt = pnlcloud->points[i];
    if ( minvec[0] > tpt.x ) minvec[0] = tpt.x;
    if ( minvec[1] > tpt.y ) minvec[1] = tpt.y;
    if ( minvec[2] > tpt.z ) minvec[2] = tpt.z;
    if ( maxvec[0] < tpt.x ) maxvec[0] = tpt.x;
    if ( maxvec[1] < tpt.y ) maxvec[1] = tpt.y;
    if ( maxvec[2] < tpt.z ) maxvec[2] = tpt.z;
  }
#endif

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::Normal>::Ptr normalcloud( new pcl::PointCloud<pcl::Normal>() );
  split_pointnormall_cloud( pnlcloud, xyzcloud, normalcloud );

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(xyzcloud, rand()%255, rand()%255, rand()%255);
  viewer->addPointCloud<pcl::PointXYZ> (xyzcloud, color, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (xyzcloud, normalcloud, 10, 0.02, "normal cloud");
  viewer->addCoordinateSystem (0.05, -0.016998, 0.110863, -0.00500573 ); //-0.016998    0.110863 -0.00200573
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  // octree
//  float voxelsize = 0.02f;
//  pcl::octree::OctreePointCloud<PointNormalL> octree(voxelsize);
//  octree.setInputCloud (pnlcloud);
//  octree.defineBoundingBox( minvec[0], minvec[1], minvec[2], maxvec[0], maxvec[1], maxvec[2] );
//  octree.addPointsFromInputCloud ();





  return 1;
}

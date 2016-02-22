#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/lexical_cast.hpp>
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadcloud( std::string filename ) {
  std::ifstream intxt( filename.c_str() );
  std::string line;
  getline( intxt, line );
  int npoints = boost::lexical_cast<int>(line);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = npoints;
  cloud->height = 1;
  cloud->is_dense = false;
  for ( int i = 0; i < npoints; ++ i ) {
    getline(intxt, line);
//    cout << line;
    std::vector<std::string> segs;
    boost::algorithm::split( segs, line, boost::algorithm::is_any_of(" ") );
//    for ( int j = 0; j < 3; ++ j )
//      cout << segs[j] << " ";
//    cout << endl;

    pcl::PointXYZ pt;
    pt.x = boost::lexical_cast<float>(segs[0]);
    pt.y = boost::lexical_cast<float>(segs[1]);
    pt.z = boost::lexical_cast<float>(segs[2]);
    cloud->points.push_back( pt );

//    cout << pt.x << ", " << pt.y << ", " << pt.z << endl;
//    getchar();
  }
  return cloud;
}
template<int N>
struct Pt {
  float p[N];


  Pt<N>& init(float *pt) { memcpy( p, pt, N*sizeof(float) ); return *this; }

  template<typename T> Pt<N>& init(T p0) { p[0]=p0; return *this; };
  template<typename T> Pt<N>& init(T p0, T p1) { p[0]=p0; p[1]=p1; return *this; };
  template<typename T> Pt<N>& init(T p0, T p1, T p2 ) { p[0]=p0; p[1]=p1; p[2]=p2; return *this; };
  template<typename T> Pt<N>& init(T p0, T p1, T p2, T p3 ) { p[0]=p0; p[1]=p1; p[2]=p2; p[3]=p3; return *this; };

  float& operator[] (int n) { return p[n]; }
  const float& operator[] (int n) const { return p[n]; }

  operator float *() { return p; }
  operator const float *() const { return p; }

  bool operator< ( const Pt<N> &pt ) const { for(int x=0; x<N; x++) if((*this)[x]!=pt[x]) return (*this)[x]<pt[x]; return false; }
  bool operator==( const Pt<N> &pt ) const { for(int x=0; x<N; x++) if((*this)[x]!=pt[x]) return false; return true; }

  Pt<N>& operator*= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]*=pt[x]; return *this; }
  Pt<N>& operator/= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]/=pt[x]; return *this; }
  Pt<N>& operator+= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]+=pt[x]; return *this; }
  Pt<N>& operator-= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]-=pt[x]; return *this; }

  template<typename T> Pt<N>& operator*= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]*=f; return *this; }
  template<typename T> Pt<N>& operator/= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]/=f; return *this; }
  template<typename T> Pt<N>& operator+= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]+=f; return *this; }
  template<typename T> Pt<N>& operator-= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]-=f; return *this; }

  template<typename T> Pt<N> operator*( const T &pt ) const {  Pt<N> r=*this; return r*=pt; }
  template<typename T> Pt<N> operator/( const T &pt ) const {  Pt<N> r=*this; return r/=pt; }
  template<typename T> Pt<N> operator+( const T &pt ) const {  Pt<N> r=*this; return r+=pt; }
  template<typename T> Pt<N> operator-( const T &pt ) const {  Pt<N> r=*this; return r-=pt; }

  float sqEuclDist( const Pt<N> &pt ) const {	float d, r=0; for(int x=0; x<N; r+=d*d, x++) d=pt[x]-(*this)[x]; return r; }
  float euclDist( const Pt<N> &pt ) const { return sqrt(sqEuclDist(pt)); }
  Pt<N>& norm() { float d=0; for(int x=0; x<N; x++) d+=(*this)[x]*(*this)[x]; d=1./sqrt(d); for(int x=0; x<N; x++) (*this)[x]*=d; return *this; }

  friend ostream& operator<< (ostream &out, const Pt<N> &pt) { out<<"["; for(int x=0; x<N; x++) out<<(x==0?"":" ")<<pt[x]; out<<"]"; return out; }
  friend istream& operator>> (istream &in, Pt<N> &pt) { for(int x=0; x<N; x++) in>>pt[x]; return in; }

  Pt<N> &min( Pt<N> &p2) { for(int x=0; x<N; x++) (*this)[x]=std::min((*this)[x], p2[x]); return *this;}
  Pt<N> &max( Pt<N> &p2) { for(int x=0; x<N; x++) (*this)[x]=std::max((*this)[x], p2[x]); return *this;}

};

int main( int argc, char ** argv ) {
  string cloudpath = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud = loadcloud( cloudpath );
  cout << "cloud size = " << cloud->size() << endl;

//  pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  float leafsize = 0.02f;
  vg.setLeafSize(leafsize, leafsize, leafsize);
  vg.setInputCloud(cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr fcloud( new pcl::PointCloud<pcl::PointXYZ>() );
  vg.filter( *fcloud );
  cout << "filter cloud size = " << fcloud->size() << endl;
  cout << "number of voxels on axis: " << vg.getNrDivisions().transpose() << endl;
  cout << "min voxel coordinate: " << vg.getMinBoxCoordinates().transpose() << endl;
  cout << "max voxel coordinate: " << vg.getMaxBoxCoordinates().transpose() << endl;
  map< Pt<3>, int > valid_voxels;
  for ( int i = 0; i < fcloud->size(); ++ i ) {
    Eigen::Vector3i voxelid = vg.getGridCoordinates( fcloud->points[i].x, fcloud->points[i].y, fcloud->points[i].z );
    Pt<3> pt; pt.init( voxelid(0), voxelid(1), voxelid(2) );
    valid_voxels.insert( make_pair(pt, i) );
  }
  typedef map<Pt<3>, int>::iterator VoxIt;
  for ( VoxIt it = valid_voxels.begin(); it != valid_voxels.end(); ++ it ) {
    cout << it->first << ", " << it->second << endl;
  }


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
  ofstream voxelout( "voxel.txt" );
  viewer->setBackgroundColor (0, 0, 0);
  for ( int i = 0; i < fcloud->size(); ++ i ) {
    pcl::PointXYZ & pt = fcloud->points[i];
    Eigen::Vector3i voxelid = vg.getGridCoordinates( pt.x, pt.y, pt.z );
    int x = voxelid(0); int y = voxelid(1); int z = voxelid(2);
    float minz = z*leafsize; float miny = y*leafsize; float minx = x*leafsize;
    float maxz = (z+1)*leafsize; float maxy = (y+1)*leafsize; float maxx = (x+1)*leafsize;
    int iid = z*100+y*10+x;
//    viewer->addLine( pcl::PointXYZ(minx, miny, minz), pcl::PointXYZ(maxx, maxy, maxz), boost::lexical_cast<string>(i) );
    voxelout << minx << " " << miny << " " << minz << " " << maxx << " " << maxy << " " << maxz << endl;
    viewer->addCube( minx, maxx, miny, maxy, minz, maxz, 1.0, 1.0, 1.0, boost::lexical_cast<string>(iid) );
  }
  voxelout.close();

  ofstream ptvoxelidout( "ptvoxelind.txt" );
  for ( int i = 0; i < cloud->points.size(); ++ i ) {
    pcl::PointXYZ & pt = cloud->points[i];
    Eigen::Vector3i voxelid = vg.getGridCoordinates( pt.x, pt.y, pt.z );
    Pt<3> pt3d; pt3d.init( voxelid(0), voxelid(1), voxelid(2) );
    VoxIt it = valid_voxels.find(pt3d);
    int vind;
    if ( it != valid_voxels.end() ) {
      vind = it->second;
    }
    ptvoxelidout << vind << endl;
  }
  ptvoxelidout.close();

  ofstream voxellabelout("voxellabel.txt");
  Eigen::Vector2f origvec; origvec(0) = 1.0; origvec(1) = 0.0;
  for ( int i = 0; i < fcloud->size(); ++ i ) {
    pcl::PointXYZ & pt = fcloud->points[i];
    Eigen::Vector3i voxelid = vg.getGridCoordinates( pt.x, pt.y, pt.z );
    int x = voxelid(0); int y = voxelid(1); int z = voxelid(2);
    float ctrz = z*leafsize+leafsize/2; float ctry = y*leafsize+leafsize/2; float ctrx = x*leafsize+leafsize/2;
    Eigen::Vector2f ctrvec;
    ctrvec(0) = ctrx; ctrvec(1) = ctrz;
    float angle = ( atan2(ctrvec(1), ctrvec(0)) )*180/M_PI;
    if ( angle < 0 ) {
      angle += 360;
    }
    int voxellabel = int(angle)/45;
    voxellabelout << voxellabel << endl;
//    cout << ctrvec.transpose() << " -> " << angle << " label " << voxellabel << endl;
  }
  voxellabelout.close();


  viewer->addPointCloud<pcl::PointXYZ> (fcloud, "fcloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "fcloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 1;
}


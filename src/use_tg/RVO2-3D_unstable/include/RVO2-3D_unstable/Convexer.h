#include "HACD_Lib/hacdHACD.h"
#include <pqp/PQP.h>
#include <pqp/Tri.h>
#include <vector>

// Static obstacles includes
// #include <assimp/assimp.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace RVO_UNSTABLE {
  
struct ConvParam {
  ConvParam() {
    nClusters = 2;
    concavity = 5.0;
    addExtraDistPoints = true;
    addFacesPoints =  false;
    ccConnectDist = 0.1;
    targetNTrianglesDecimatedMesh = 2000;
    addNeighboursDistPoints = false;
  }
  
  size_t nClusters;
  double concavity;
  bool addExtraDistPoints;
  bool addNeighboursDistPoints;
  bool addFacesPoints;
  double ccConnectDist;
  size_t targetNTrianglesDecimatedMesh;
};

class Convexer {
public:
  static std::vector<PQP_Model *> loadScenario(const std::string& file, bool make_convex = false);
  
  static std::vector<PQP_Model *> fromaiScene(const aiScene *scene);
  
  static std::vector<PQP_Model *> makeConvex(const aiScene *scene);
  
  static void toHACD(const aiMesh *mesh, HACD::HACD *myHACD);
  
  static void makeConvex(HACD::HACD* myHACD, std::vector< PQP_Model* >& ret_val, const RVO_UNSTABLE::ConvParam& p = ConvParam() 
 					      );
  
  static void addModels(HACD::HACD *myHACD, std::vector<PQP_Model *> &mod_vec);
  
  static bool SavePartition(const std::string & fileName, const std::vector< HACD::Vec3<HACD::Real> > & points, 
                                                 const std::vector< HACD::Vec3<long> > & triangles,
                                                 const long * partition, const size_t nClusters);
  
  static bool SaveVRML2(std::ofstream & fout, const std::vector< HACD::Vec3<HACD::Real> > & points, 
               const std::vector< HACD::Vec3<long> > & triangles, 
               const HACD::Material & material, const HACD::Vec3<HACD::Real> * colors = NULL);
  
  
};

};
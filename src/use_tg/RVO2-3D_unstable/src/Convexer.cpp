#include "Convexer.h"
#include <iostream>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include "HACD_Lib/hacdHACD.h"

using std::cout;
using std::endl;

namespace RVO_UNSTABLE {

std::vector<PQP_Model *> Convexer::loadScenario(const std::string& file, bool make_convex)
{
  const aiScene* scene = NULL;
  std::vector<PQP_Model *> ret_val;
  // Create an instance of the Importer class
  Assimp::Importer importer;
  // And have it read the given file with some example postprocessing
  // Usually - if speed is not the most important aspect for you - you'll 
  // propably to request more postprocessing than we do in this example.
  scene = importer.ReadFile( file, 
        aiProcess_CalcTangentSpace       | 
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);
  
  // If the import failed, report it
  if( !scene)
  {
    std::cerr << "Convexer::loadScenario --> " <<  importer.GetErrorString() << std::endl;
  } else {
    std::cout << "Convexer::loadScenario --> Scene loaded successfully. Num meshes: ";
    std::cout << scene->mNumMeshes << std::endl;
    if (make_convex) {
      cout << "\nConvexer::loadScenario --> Making convex the mesh.\n";
      ret_val = makeConvex(scene);
    } else {
      ret_val = fromaiScene(scene);
    }
  }
  
  cout << "Convexer::loadScenario --> Scene loaded contains " << ret_val.size() << " different obstacles.\n";
  
  // We're done. Everything will be cleaned up by the importer destructor
  return ret_val;
}

std::vector<PQP_Model *> Convexer::fromaiScene(const aiScene *scene) {
  // Get the triangles
  std::vector<PQP_Model *> ret_val;
  PQP_REAL v1[3], v2[3], v3[3], v4[3];
  for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
    aiMesh *curr_mesh = scene->mMeshes[i];
    if (curr_mesh != NULL) {
      PQP_Model *mod = new PQP_Model();
      for (unsigned int k = 0; k < curr_mesh->mNumFaces; k++) {
	aiFace &curr_face = curr_mesh->mFaces[k];
	
	if (curr_face.mNumIndices == 3) {
	  // Triangualr faces!! --> ok (TODO: triangulate the non-triangular faces
	  v1[0] = curr_mesh->mVertices[curr_face.mIndices[0]].x;
	  v1[1] = curr_mesh->mVertices[curr_face.mIndices[0]].y;
	  v1[2] = curr_mesh->mVertices[curr_face.mIndices[0]].z;
	  v2[0] = curr_mesh->mVertices[curr_face.mIndices[1]].x;
	  v2[1] = curr_mesh->mVertices[curr_face.mIndices[1]].y;
	  v2[2] = curr_mesh->mVertices[curr_face.mIndices[1]].z;
	  v3[0] = curr_mesh->mVertices[curr_face.mIndices[2]].x;
	  v3[1] = curr_mesh->mVertices[curr_face.mIndices[2]].y;
	  v3[2] = curr_mesh->mVertices[curr_face.mIndices[2]].z;
	  
	  // Debug
// 	  std::cout << "Adding triangle: (" << v1[0] << ", " << v1[1] << ", " << v1[2] << ") ";
// 	  std::cout << "(" << v2[0] << ", " << v2[1] << ", " << v2[2] << ") ";
// 	  std::cout << "(" << v3[0] << ", " << v3[1] << ", " << v3[2] << ")" << std::endl;
	  
	  mod->AddTri(v1, v2, v3, k);
	} else if (curr_face.mNumIndices == 4) {
	  v1[0] = curr_mesh->mVertices[curr_face.mIndices[0]].x;
	  v1[1] = curr_mesh->mVertices[curr_face.mIndices[0]].y;
	  v1[2] = curr_mesh->mVertices[curr_face.mIndices[0]].z;
	  v2[0] = curr_mesh->mVertices[curr_face.mIndices[1]].x;
	  v2[1] = curr_mesh->mVertices[curr_face.mIndices[1]].y;
	  v2[2] = curr_mesh->mVertices[curr_face.mIndices[1]].z;
	  v3[0] = curr_mesh->mVertices[curr_face.mIndices[2]].x;
	  v3[1] = curr_mesh->mVertices[curr_face.mIndices[2]].y;
	  v3[2] = curr_mesh->mVertices[curr_face.mIndices[2]].z;
	  v4[0] = curr_mesh->mVertices[curr_face.mIndices[3]].x;
	  v4[1] = curr_mesh->mVertices[curr_face.mIndices[3]].y;
	  v4[2] = curr_mesh->mVertices[curr_face.mIndices[3]].z;
	  mod->AddTri(v1, v2, v3, k);
	  mod->AddTri(v1, v4, v3, k);
// 	  std::cout << "Adding quadrangle: (" << v1[0] << ", " << v1[1] << ", " << v1[2] << ") ";
// 	  std::cout << "(" << v2[0] << ", " << v2[1] << ", " << v2[2] << ") ";
// 	  std::cout << "(" << v3[0] << ", " << v3[1] << ", " << v3[2] << ")" << std::endl;
	} else {
	  std::cerr << "Warning faces with more than 4 edges are not allowed at this moment.\n";
	}
      }
      if (curr_mesh->mNumFaces > 0 && mod->num_tris > 0) {
	mod->EndModel();
	ret_val.push_back(mod);
      }
    }
  }
  
  return ret_val;
}

std::vector<PQP_Model *> Convexer::makeConvex(const aiScene *scene) {
  HACD::HeapManager * heapManager = HACD::createHeapManager(65536*(1000));
  std::vector<PQP_Model *> ret_val;
  
  for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
    cout << "makeConvex --> mesh " << i + 1 << endl;
    aiMesh *curr_mesh = scene->mMeshes[i];
    HACD::HACD * myHACD = HACD::CreateHACD(heapManager);
    
    // Convert the mesh
    toHACD(curr_mesh, myHACD);
    makeConvex(myHACD, ret_val);
    HACD::DestroyHACD(myHACD);
  }
  
    HACD::releaseHeapManager(heapManager);
  
  return  ret_val;
}

void Convexer::makeConvex(HACD::HACD * myHACD, std::vector<PQP_Model *> &ret_val, const ConvParam &p) {
  
  
  // Set the approximation parameters
  myHACD->SetCompacityWeight(0.001);
  myHACD->SetVolumeWeight(0.0);
  myHACD->SetConnectDist(p.ccConnectDist);               // if two connected components are seperated by distance < ccConnectDist
						      // then create a virtual edge between them so the can be merged during 
						      // the simplification process
	    
  myHACD->SetNClusters(p.nClusters);                     // minimum number of clusters
  myHACD->SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
  myHACD->SetConcavity(p.concavity);                     // maximum concavity
  myHACD->SetSmallClusterThreshold(0.25);				 // threshold to detect small clusters
  myHACD->SetNTargetTrianglesDecimatedMesh(p.targetNTrianglesDecimatedMesh); // # triangles in the decimated mesh
//     myHACD->SetCallBack(&CallBack);
  myHACD->SetAddExtraDistPoints(p.addExtraDistPoints);   
  myHACD->SetAddFacesPoints(p.addFacesPoints); 
  
  myHACD->Compute(); // Make it Convex!!
  
  addModels(myHACD, ret_val);
  
  // Debug
//   cout << "Convex process ended successfully. Number of models: " << ret_val.size() << endl;
//   cout << "Model 0. Num triangles:  " << ret_val.at(0)->num_tris << endl;;
//   for (unsigned int i = 0; i < ret_val.at(0)->num_tris ;i++) {
//     cout << "Triangle " << i << " (";
//     cout << ret_val.at(0)->tris[i].p1[0] << ", " << ret_val.at(0)->tris[i].p1[1] << ", " << ret_val.at(0)->tris[i].p1[2] << ") (";
//     cout << ret_val.at(0)->tris[i].p2[0] << ", " << ret_val.at(0)->tris[i].p2[1] << ", " << ret_val.at(0)->tris[i].p2[2] << ") (";
//     cout << ret_val.at(0)->tris[i].p3[0] << ", " << ret_val.at(0)->tris[i].p3[1] << ", " << ret_val.at(0)->tris[i].p3[2] << ").  ";
    
//   }
}

void Convexer::toHACD(const aiMesh *mesh, HACD::HACD *myHACD) {
  HACD::Vec3<HACD::Real> *points = new HACD::Vec3<HACD::Real>[mesh->mNumVertices];
  HACD::Vec3<long> *triangles = new HACD::Vec3<long>[mesh->mNumFaces];
  for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
    points[i][0] = mesh->mVertices[i].x;
    points[i][1] = mesh->mVertices[i].y;
    points[i][2] = mesh->mVertices[i].z;
  }
  for (unsigned int k = 0; k < mesh->mNumFaces; k++) {
    aiFace &curr_face = mesh->mFaces[k];
    
    if (curr_face.mNumIndices == 3) {
      for (unsigned int i = 0; i < curr_face.mNumIndices; i++) {
	triangles[k][i] = curr_face.mIndices[i];
      }
    } else{
      std::cout << "error\n";
    }
  }
  myHACD->SetPoints(&points[0]);
  myHACD->SetTriangles(&triangles[0]);
  myHACD->SetNPoints(mesh->mNumVertices);
  myHACD->SetNTriangles(mesh->mNumFaces);
  
  myHACD->Save("/home/sinosuke/original.wrl", false);
}

void Convexer::addModels(HACD::HACD *myHACD, std::vector<PQP_Model *> &mod_vec){
  
  
  PQP_REAL v[3][3];
  
  cout << "addModels --> Num clusters: " << myHACD->GetNClusters() << endl;
  
  myHACD->Save("/home/sinosuke/out.wrl", false);
  
  for(size_t c = 0; c < myHACD->GetNClusters(); ++c) {
    cout << "addModels --> cluster " << c << endl;
    PQP_Model *mod = new PQP_Model();
    unsigned int id = 0;
    
    // Get the cluster
    size_t nPoints = myHACD->GetNPointsCH(c);
    size_t nTriangles = myHACD->GetNTrianglesCH(c);
    HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
    HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
    myHACD->GetCH(c, pointsCH, trianglesCH);
    
    for (unsigned int i = 0; i < nTriangles; i++) {
//       cout << "addModels --> Adding triangle: " ;
//       
//       for (int j = 0; j < 3; j++) {
// 	cout << "(";
// 	for (int k = 0; k < 3; k++) {
// 	  v[j][k] = pointsCH[trianglesCH[i][j]][k];
// 	  cout << v[j][k] << " ";
// 	}
// 	cout << ") ";
//       }
      mod->AddTri(v[0], v[1], v[2], id);
      
      id++;
    }
    cout <<endl;
    mod->EndModel();
    mod_vec.push_back(mod);
    delete[] pointsCH;
    delete[] trianglesCH;
  }
}

bool Convexer::SavePartition(const std::string & fileName, const std::vector< HACD::Vec3<HACD::Real> > & points, 
                                                 const std::vector< HACD::Vec3<long> > & triangles,
                                                 const long * partition, const size_t nClusters)
{
    if (!partition)
    {
        return false;
    }
    
    std::cout << "Saving " <<  fileName << std::endl;
    std::ofstream fout(fileName.c_str());
    if (fout.is_open()) 
    {
        HACD::Material mat;
        std::vector< HACD::Vec3<long> > triCluster;
        std::vector< HACD::Vec3<HACD::Real> > ptsCluster;
        std::vector< long > ptsMap;
        for(size_t c = 0; c < nClusters; c++)
        {
            ptsMap.resize(points.size());
            mat.m_diffuseColor.X() = mat.m_diffuseColor.Y() = mat.m_diffuseColor.Z() = 0.0;
            while (mat.m_diffuseColor.X() == mat.m_diffuseColor.Y() ||
                   mat.m_diffuseColor.Z() == mat.m_diffuseColor.Y() ||
                   mat.m_diffuseColor.Z() == mat.m_diffuseColor.X()  )
            {
                mat.m_diffuseColor.X() = (rand()%100) / 100.0;
                mat.m_diffuseColor.Y() = (rand()%100) / 100.0;
                mat.m_diffuseColor.Z() = (rand()%100) / 100.0;
            }
            long ver[3];
            long vCount = 1;
            for(size_t t = 0; t < triangles.size(); t++)
            {
                if (partition[t] == static_cast<long>(c))
                {
                    ver[0] = triangles[t].X();
                    ver[1] = triangles[t].Y();
                    ver[2] = triangles[t].Z();
                    for(int k = 0; k < 3; k++)
                    {
                        if (ptsMap[ver[k]] == 0)
                        {
                            ptsCluster.push_back(points[ver[k]]);
                            ptsMap[ver[k]] = vCount;
                            ver[k] = vCount-1;
                            vCount++;
                        }
                        else
                        {
                            ver[k] = ptsMap[ver[k]]-1;
                        }
                    }
                    triCluster.push_back(HACD::Vec3<long>(ver[0], ver[1], ver[2]));
                }
            }
            SaveVRML2(fout, ptsCluster, triCluster, mat);
            triCluster.clear();
            ptsCluster.clear();
            ptsMap.clear();
        }

        fout.close();
        return true;
    }
    return false;    
}

bool Convexer::SaveVRML2(std::ofstream & fout, const std::vector< HACD::Vec3<HACD::Real> > & points, 
               const std::vector< HACD::Vec3<long> > & triangles, 
               const HACD::Material & material, const HACD::Vec3<HACD::Real> * colors)
{
    if (fout.is_open()) 
    {
        size_t nV = points.size();
        size_t nT = triangles.size();            
        fout <<"#VRML V2.0 utf8" << std::endl;	    	
        fout <<"" << std::endl;
        fout <<"# Vertices: " << nV << std::endl;		
        fout <<"# Triangles: " << nT << std::endl;		
        fout <<"" << std::endl;
        fout <<"Group {" << std::endl;
        fout <<"	children [" << std::endl;
        fout <<"		Shape {" << std::endl;
        fout <<"			appearance Appearance {" << std::endl;
        fout <<"				material Material {" << std::endl;
        fout <<"					diffuseColor "      << material.m_diffuseColor.X()      << " " 
                                                        << material.m_diffuseColor.Y()      << " "
                                                        << material.m_diffuseColor.Z()      << std::endl;  
        fout <<"					ambientIntensity "  << material.m_ambientIntensity      << std::endl;
        fout <<"					specularColor "     << material.m_specularColor.X()     << " " 
                                                        << material.m_specularColor.Y()     << " "
                                                        << material.m_specularColor.Z()     << std::endl; 
        fout <<"					emissiveColor "     << material.m_emissiveColor.X()     << " " 
                                                        << material.m_emissiveColor.Y()     << " "
                                                        << material.m_emissiveColor.Z()     << std::endl; 
        fout <<"					shininess "         << material.m_shininess             << std::endl;
        fout <<"					transparency "      << material.m_transparency          << std::endl;
        fout <<"				}" << std::endl;
        fout <<"			}" << std::endl;
        fout <<"			geometry IndexedFaceSet {" << std::endl;
        fout <<"				ccw TRUE" << std::endl;
        fout <<"				solid TRUE" << std::endl;
        fout <<"				convex TRUE" << std::endl;
        if (colors && nT>0)
        {
            fout <<"				colorPerVertex FALSE" << std::endl;
            fout <<"				color Color {" << std::endl;
            fout <<"					color [" << std::endl;
            for(size_t c = 0; c < nT; c++)
            {
                fout <<"						" << colors[c].X() << " " 
                                                  << colors[c].Y() << " " 
                                                  << colors[c].Z() << "," << std::endl;
            }
            fout <<"					]" << std::endl;
            fout <<"				}" << std::endl;
                    }
        if (nV > 0) 
        {
            fout <<"				coord DEF co Coordinate {" << std::endl;
            fout <<"					point [" << std::endl;
            for(size_t v = 0; v < nV; v++)
            {
                fout <<"						" << points[v].X() << " " 
                                                  << points[v].Y() << " " 
                                                  << points[v].Z() << "," << std::endl;
            }
            fout <<"					]" << std::endl;
            fout <<"				}" << std::endl;
        }
        if (nT > 0) 
        {
            fout <<"				coordIndex [ " << std::endl;
            for(size_t f = 0; f < nT; f++)
            {
                fout <<"						" << triangles[f].X() << ", " 
                                                  << triangles[f].Y() << ", "                                                  
                                                  << triangles[f].Z() << ", -1," << std::endl;
            }
            fout <<"				]" << std::endl;
        }
        fout <<"			}" << std::endl;
        fout <<"		}" << std::endl;
        fout <<"	]" << std::endl;
        fout <<"}" << std::endl;	
	    return true;
    }
    return false;
}


} // namespace RVO_UNSTABLE

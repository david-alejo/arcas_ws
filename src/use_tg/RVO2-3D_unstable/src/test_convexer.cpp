#include <functions/ArgumentData.h>
#include <functions/functions.h>
#include "RVO2-3D_unstable/Convexer.h"
#include "HACD_Lib/hacdHACD.h"
#include <iostream>
#include <boost/concept_check.hpp>
#include <qhull/Qhull.h>

using functions::ArgumentData;
using namespace RVO_UNSTABLE;

bool LoadOFF(const std::string & fileName, 
	     std::vector< HACD::Vec3<HACD::Real> > & points, 
	     std::vector< HACD::Vec3<long> > & triangles, 
	     bool invert = false);

bool SaveOFF(const std::string & fileName, 
	     const std::vector< HACD::Vec3<HACD::Real> > & points, 
	     const std::vector< HACD::Vec3<long> > & triangles);

bool SaveOFF(const std::string & fileName, 
	     size_t nV, 
	     size_t nT, 
	     const HACD::Vec3<HACD::Real> * const points, 
	     const HACD::Vec3<long> * const triangles);

// orgQhull::RboxPoints getRboxPoints(const std::vector<HACD::Vec3<HACD::Real> >& points);

int main(int argc, char**argv) {
  ArgumentData arg(argc, argv);
  
  if (arg.size() < 2) {
    std::cerr << "Use: " << arg[0] << " <input file> <output file>\n";
    return -1;
  }
  
  std::vector< HACD::Vec3<HACD::Real> >  points;
  std::vector< HACD::Vec3<long> > triangles;
  
  bool error_;

  
  
  if (!error_) {
    
    HACD::HeapManager * heapManager = HACD::createHeapManager(65536*(1000));

    HACD::HACD * const myHACD = HACD::CreateHACD(heapManager);
    myHACD->SetPoints(&points[0]);
    myHACD->SetNPoints(points.size());
    myHACD->SetTriangles(&triangles[0]);
    myHACD->SetNTriangles(triangles.size());
      
    if (arg.isOption("normal")) {
      error_ = Convexer::loadScenario(arg[1], true).size() == 0;
    } else {
      error_ = !LoadOFF(arg[1], points, triangles);
    }
  
    
    std::vector<PQP_Model *> ret;
    
    ConvParam p;
    
    if (arg.isOption("concavity")) {
      arg.getOption("concavity", p.concavity);
      std::cout << "Setting concavity param to " << p.concavity << std::endl;
    }
    
    if (arg.isOption("triangles_decimated")) {
      arg.getOption("triangles_decimated", p.targetNTrianglesDecimatedMesh);
      std::cout << "Setting targetNTrianglesDecimatedMesh param to " << p.targetNTrianglesDecimatedMesh << std::endl;
    }
    
    
    
    Convexer::makeConvex(myHACD, ret, p);
      
    
    std::string outFileName = arg[1] + "_hacd.wrl";
    myHACD->Save(outFileName.c_str(), false);
      
    const HACD::Vec3<HACD::Real> * const decimatedPoints = myHACD->GetDecimatedPoints();
    const HACD::Vec3<long> * const decimatedTriangles    = myHACD->GetDecimatedTriangles();
    if (decimatedPoints && decimatedTriangles)
    {
      std::cout << "Saving the results to: " << arg[2] << std::endl;
      SaveOFF(arg[2], myHACD->GetNDecimatedPoints(), 
					  myHACD->GetNDecimatedTriangles(), decimatedPoints, decimatedTriangles);
    
    
      //     TODO: Make this right
      const long * const partition = myHACD->GetPartition();
      Convexer::SavePartition("partition.vrml", points, triangles, partition, myHACD->GetNClusters());
    }
    // TODO: Convex Hull 
    if (arg.isOption("convex_hull")) {
      std::string s;
      arg.getOption("convex_hull", s);
      std::cout << "Exporting convex hull to: " << s << std::endl;
      
//       orgQhull::Qhull h(getRboxPoints(points), "");
//       
//       myHACD->Save(s.c_str(), false);
    }
    
    if (arg.isOption("text")) {
      std::string s;
      arg.getOption("text", s);
      std::cout << "Exporting text file to: " << s << std::endl;
      
      std::ostringstream os;
      os << "3 " << points.size() << std::endl;
      for (int i = 0; i < points.size(); i++) {
	os << points.at(i)[0] << " ";
	os << points.at(i)[1] << " ";
	os << points.at(i)[2] << std::endl;
      }
      if (!functions::writeStringToFile(s, os.str())) {
	std::cerr << "Could not write the text file " << s << std::endl;
      } else {
	std::cout << "Text file: " << s << " written successfully\n";
      }
      
//       myHACD->Save(s.c_str(), false);
    }
    
  } else {
    std::cerr << "Could not open file " << arg[1] << std::endl;
    return -1;
  }
  
//   Convexer::SaveVRML2()

  return 0;
}

bool LoadOFF(const std::string & fileName, std::vector< HACD::Vec3<HACD::Real> > & points, std::vector< HACD::Vec3<long> > & triangles, bool invert) 
{    
	FILE * fid = fopen(fileName.c_str(), "r");
	if (fid) 
    {
		const std::string strOFF("OFF");
		char temp[1024];
		fscanf(fid, "%s", temp);
		if (std::string(temp) != strOFF)
		{
			printf( "Loading error: format not recognized \n");
            fclose(fid);

			return false;            
		}
		else
		{
			int nv = 0;
			int nf = 0;
			int ne = 0;
			fscanf(fid, "%i", &nv);
			fscanf(fid, "%i", &nf);
			fscanf(fid, "%i", &ne);
            points.resize(nv);
			triangles.resize(nf);
            HACD::Vec3<HACD::Real> coord;
			float x = 0;
			float y = 0;
			float z = 0;
			for (long p = 0; p < nv ; p++) 
            {
				fscanf(fid, "%f", &x);
				fscanf(fid, "%f", &y);
				fscanf(fid, "%f", &z);
				points[p].X() = x;
				points[p].Y() = y;
				points[p].Z() = z;
			}        
			int i = 0;
			int j = 0;
			int k = 0;
			int s = 0;
			for (long t = 0; t < nf ; ++t) {
				fscanf(fid, "%i", &s);
				if (s == 3)
				{
					fscanf(fid, "%i", &i);
					fscanf(fid, "%i", &j);
					fscanf(fid, "%i", &k);
					triangles[t].X() = i;
					if (invert)
					{
						triangles[t].Y() = k;
						triangles[t].Z() = j;
					}
					else
					{
						triangles[t].Y() = j;
						triangles[t].Z() = k;
					}
				}
				else			// Fix me: support only triangular meshes
				{
					for(long h = 0; h < s; ++h) fscanf(fid, "%i", &s);
				}
			}
            fclose(fid);
		}
	}
	else 
    {
		printf( "Loading error: file not found \n");
		return false;
    }
	return true;
}

bool SaveOFF(const std::string & fileName, const std::vector< HACD::Vec3<HACD::Real> > & points, const std::vector< HACD::Vec3<long> > & triangles)
{
	return SaveOFF(fileName,points.size(), triangles.size(), &points[0], &triangles[0]);
}
bool SaveOFF(const std::string & fileName, size_t nV, size_t nT, const HACD::Vec3<HACD::Real> * const points, const HACD::Vec3<long> * const triangles)
{
    std::cout << "Saving " <<  fileName << std::endl;
    std::ofstream fout(fileName.c_str());
    if (fout.is_open()) 
    {           
        fout <<"OFF" << std::endl;	    	
        fout << nV << " " << nT << " " << 0<< std::endl;		
        for(size_t v = 0; v < nV; v++)
        {
            fout << points[v].X() << " " 
                 << points[v].Y() << " " 
                 << points[v].Z() << std::endl;
		}
        for(size_t f = 0; f < nT; f++)
        {
            fout <<"3 " << triangles[f].X() << " " 
                        << triangles[f].Y() << " "                                                  
                        << triangles[f].Z() << std::endl;
        }
        fout.close();
	    return true;
    }
    return false;
}

// orgQhull::RboxPoints getRboxPoints(const std::vector<HACD::Vec3<HACD::Real> >&points) {
//   std::vector<double> v;
//   
//   for (unsigned int i = 0; i < points.size(); i++) {
//     
//   }
//   
//   return orgQhull::RboxPoints();
// }

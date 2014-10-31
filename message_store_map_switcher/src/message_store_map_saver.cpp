#include "mongodb_store/message_store.h"
#include "map_server/image_loader.h"

#include "ros/ros.h"

#include "yaml-cpp/yaml.h"
#include <libgen.h>
#include <fstream>

//copied from https://github.com/ros-planning/navigation/blob/hydro-devel/map_server
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif


using namespace mongodb_store;



#define USAGE "\nUSAGE: message_store_map_saver <map.yaml>\n" \
              "  map.yaml: map description file\n"

/**
 	Load a map from a yaml description. Code copied from main.cc in map_server.
*/
void loadMapFromFile(ros::NodeHandle & private_nh, 
						const std::string & fname, 
						nav_msgs::GetMap::Response & mapResponse) {

	std::string mapfname;   
	double origin[3];
	int negate;
	double occ_th, free_th;
	std::string frame_id;	
	private_nh.param("frame_id", frame_id, std::string("map"));
	double res;

	//mapfname = fname + ".pgm";
	//std::ifstream fin((fname + ".yaml").c_str());
	std::ifstream fin(fname.c_str());
	if (fin.fail()) {
	  ROS_ERROR("message_store_map_saver could not open %s.", fname.c_str());
	  exit(-1);
	}


#ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
	
	try { 
	  doc["resolution"] >> res; 
	} catch (YAML::InvalidScalar) { 
	  ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
	  exit(-1);
	}

	try { 
	  doc["negate"] >> negate; 
	} catch (YAML::InvalidScalar) { 
	  ROS_ERROR("The map does not contain a negate tag or it is invalid.");
	  exit(-1);
	}
	
	try { 
	  doc["occupied_thresh"] >> occ_th; 
	} catch (YAML::InvalidScalar) { 
	  ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
	  exit(-1);
	}
	
	try { 
	  doc["free_thresh"] >> free_th; 
	} catch (YAML::InvalidScalar) { 
	  ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
	  exit(-1);
	}
	
	try { 
	  doc["origin"][0] >> origin[0]; 
	  doc["origin"][1] >> origin[1]; 
	  doc["origin"][2] >> origin[2]; 
	} catch (YAML::InvalidScalar) { 
	  ROS_ERROR("The map does not contain an origin tag or it is invalid.");
	  exit(-1);
	}
	
	try { 

	  doc["image"] >> mapfname; 

	  // TODO: make this path-handling more robust
	  if(mapfname.size() == 0)
	  {
	    ROS_ERROR("The image tag cannot be an empty string.");
	    exit(-1);
	  }
	  if(mapfname[0] != '/')
	  {
	    // dirname can modify what you pass it
	    char* fname_copy = strdup(fname.c_str());
	    mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
	    free(fname_copy);
	  }
	} catch (YAML::InvalidScalar) { 
	  ROS_ERROR("The map does not contain an image tag or it is invalid.");
	  exit(-1);
	}

	ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
	map_server::loadMapFromFile(&mapResponse,mapfname.c_str(),res,negate,occ_th,free_th, origin);
	mapResponse.map.info.map_load_time = ros::Time::now();
	mapResponse.map.header.frame_id = frame_id;
	mapResponse.map.header.stamp = ros::Time::now();
	ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
	       mapResponse.map.info.width,
	       mapResponse.map.info.height,
	       mapResponse.map.info.resolution);

}


int main(int argc, char **argv)
{
	if(argc != 2) {
    	ROS_ERROR("%s", USAGE);
    	exit(-1);
  	}

	ros::init(argc, argv, "message_store_map_saver");
	ros::NodeHandle private_nh("~");

	//structure to store map contents in, as required by map_server::loadMapFromFile
    nav_msgs::GetMap::Response mapResponse;
 	
 	//file name of the yaml file
	std::string fname(argv[1]);   
	
	//load map into mapResponse
	loadMapFromFile(private_nh, fname, mapResponse);

	//object to store the map in the db	
	MessageStoreProxy messageStore(private_nh);

	unsigned pos = fname.find(".");
	std::string mapName = fname.substr(0,pos);
	ROS_INFO("Saving to message store as \"%s\"", mapName.c_str());

	messageStore.updateNamed(mapName, mapResponse.map, true);

	return 0;
}



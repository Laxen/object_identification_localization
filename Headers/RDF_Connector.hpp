#ifndef RDF_CONNECTOR_H
#define RDF_CONNECTOR_H

#include "JNI_Helper.hpp"

class RDF_Connector : public JNI_Helper {
	private:
		jclass main_class;

	public:
		RDF_Connector(std::string class_path);
		~RDF_Connector();

		void
		initialize(std::string serverID, std::string repoID);

		void
		set_class(std::string class_name);

		void 
		add_scene (int scene_index);

		void 
		add_cluster (int sceneIndex, int clusterIndex);

		void 
		add_current_position (int sceneIndex, std::string position);

		void 
		add_next_position (int sceneIndex, int clusterIndex, std::string position);

		void 
		add_model (int sceneIndex, int clusterIndex, std::string name, double score, std::string pose);
};

#endif

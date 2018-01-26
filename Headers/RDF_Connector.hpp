#ifndef RDF_CONNECTOR_H
#define RDF_CONNECTOR_H

#include "JNI_Helper.hpp"

class RDF_Connector : public JNI_Helper {
	private:
		jclass main_class;
		bool has_class_path = false;

	public:
		RDF_Connector();
		RDF_Connector(std::string class_path);
		~RDF_Connector();

		void
		set_class_path(std::string class_path);

		void
		set_class(std::string class_name);

		void
		initialize(std::string serverID, std::string repoID);

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

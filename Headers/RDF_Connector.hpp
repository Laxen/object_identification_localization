#ifndef RDF_CONNECTOR_H
#define RDF_CONNECTOR_H

#include "JNI_Helper.hpp"

class RDF_Connector : public JNI_Helper {
	private:
		jclass main_class;
		bool has_class_path = false;

	public:
		/**
			Create the RDF_Connector object without specifying class_path. You will need to call set_class_path before the object is usable.
		*/
		RDF_Connector();

		/**
			Initialize RDF_Connector with a class_path
			@param class_path The class path
		*/
		RDF_Connector(std::string class_path);

		/**
			Destructor that cleans up objects
		*/
		~RDF_Connector();

		/**
			Set the class_path of the VM
			@param class_path The class path
		*/
		void
		set_class_path(std::string class_path);

		/**
			Set the main class in the classpath. This is the class which functions you want to use.
			@param class_name The name of the class
		*/
		void
		set_class(std::string class_name);

		/**
			Initialize the RDF connection 
			@param serverID The URL to the server
			@param repoID The repository name
		*/
		void
		initialize(std::string serverID, std::string repoID);

		/**
			Add a scene index to the RDF database
			@param scene_index The index of the scene
		*/
		void 
		add_scene (int scene_index);

		/**
			Add a cluster to the RDF database
			@param scene_index The scene index to which the cluster should be added
			@param cluster_index The index of the cluster
		*/
		void 
		add_cluster (int scene_index, int cluster_index);

		/**
			Add the current position to the RDF database
			@param scene_index The scene index to which the current position should be added
			@param position The current position
		*/
		void 
		add_current_position (int scene_index, std::string position);

		/**
			Add the next position to the RDF database
			@param scene_index The scene index to which the next position should be added
			@param cluster_index The cluster index to which the next position should be added
			@param position The next position
		*/
		void 
		add_next_position (int scene_index, int cluster_index, std::string position);

		/**
			Add a model to the RDF database
			@param scene_index The scene index to which the model should be added
			@param cluster_index The cluster index to which the model should be added
			@param name The name of the model
			@param score The score of the model
			@param pose The pose of the model
		*/
		void 
		add_model (int scene_index, int cluster_index, std::string name, double score, std::string pose);
};

#endif

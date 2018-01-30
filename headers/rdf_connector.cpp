#include "rdf_connector.hpp"

/**
	Create the RDF_Connector object without specifying class_path. You will need to call set_class_path before the object is usable.
*/
RDF_Connector::RDF_Connector() {
}

/**
	Initialize RDF_Connector with a class_path
	@param class_path The class path
*/
RDF_Connector::RDF_Connector(std::string class_path) : JNI_Helper(class_path) {
	has_class_path = true;
}

/**
	Destructor that cleans up objects
*/
RDF_Connector::~RDF_Connector() {
	jmethodID mid = get_static_mid(main_class, "close", "()V");
	call_static_void_method(main_class, mid);
	std::cout << "Repo closed!" << std::endl;
}

/**
	Set the class_path of the VM
	@param class_path The class path
*/
void
RDF_Connector::set_class_path(std::string class_path) {
	has_class_path = true;
	JNI_Helper::initialize(class_path);
}

/**
	Set the main class in the classpath. This is the class which functions you want to use.
	@param class_name The name of the class
*/
void
RDF_Connector::set_class(std::string class_name) {
	if(!has_class_path) {
		std::cout << "ERROR: JNI class path not set!" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	main_class = get_class(class_name);
}

/**
	Initialize the RDF connection 
	@param serverID The URL to the server
	@param repoID The repository name
*/
void
RDF_Connector::initialize(std::string serverID, std::string repoID) {
	if(!has_class_path) {
		std::cout << "ERROR: JNI class path not set!" << std::endl;
		std::exit (EXIT_FAILURE);
	}

	std::cout << "Initializing RDF connection to " << serverID << " with repo name " << repoID << std::endl;

	jstring sid = create_string(serverID);
	jstring rid = create_string(repoID);

	jmethodID mid = get_static_mid(main_class, "initialize", "(Ljava/lang/String;Ljava/lang/String;)V");
	call_static_void_method(main_class, mid, sid, rid);

	mid = get_static_mid(main_class, "clearRepo", "()V");
	call_static_void_method(main_class, mid);
}

/**
	Add a scene index to the RDF database
	@param scene_index The index of the scene
*/
void 
RDF_Connector::add_scene (int scene_index) {
	jmethodID mid = get_static_mid(main_class, "addScene", "(I)V");
	call_static_void_method(main_class, mid, scene_index);
}

/**
	Add a cluster to the RDF database
	@param scene_index The scene index to which the cluster should be added
	@param cluster_index The index of the cluster
*/
void 
RDF_Connector::add_cluster (int scene_index, int cluster_index) {
	jmethodID mid = get_static_mid(main_class, "addCluster", "(II)V");
	call_static_void_method(main_class, mid, scene_index, cluster_index);
}

/**
	Add the current position to the RDF database
	@param scene_index The scene index to which the current position should be added
	@param position The current position
*/
void 
RDF_Connector::add_current_position (int scene_index, std::string position) {
	jstring pos = create_string(position);
	jmethodID mid = get_static_mid(main_class, "addCurrentPosition", "(ILjava/lang/String;)V");
	call_static_void_method(main_class, mid, scene_index, pos);
}

/**
	Add the next position to the RDF database
	@param scene_index The scene index to which the next position should be added
	@param cluster_index The cluster index to which the next position should be added
	@param position The next position
*/
void 
RDF_Connector::add_next_position (int scene_index, int cluster_index, std::string position)  {
	jstring pos = create_string(position);
	jmethodID mid = get_static_mid(main_class, "addNextPosition", "(IILjava/lang/String;)V");
	call_static_void_method(main_class, mid, scene_index, cluster_index, pos);
}

/**
	Add a model to the RDF database
	@param scene_index The scene index to which the model should be added
	@param cluster_index The cluster index to which the model should be added
	@param name The name of the model
	@param score The score of the model
	@param pose The pose of the model
*/
void 
RDF_Connector::add_model (int scene_index, int cluster_index, std::string name, double score, std::string pose) {
	jstring n = create_string(name);
	jstring p = create_string(pose);
	jmethodID mid = get_static_mid(main_class, "addModel", "(IILjava/lang/String;DLjava/lang/String;)V");
	call_static_void_method(main_class, mid, scene_index, cluster_index, n, score, p);
}

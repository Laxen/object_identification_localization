#include "RDF_Connector.hpp"

RDF_Connector::RDF_Connector(std::string class_path) : JNI_Helper(class_path) {
}

RDF_Connector::~RDF_Connector() {
	jmethodID mid = get_static_mid(main_class, "close", "()V");
	call_static_void_method(main_class, mid);
	std::cout << "Repo closed!" << std::endl;
}

void
RDF_Connector::set_class(std::string class_name) {
	main_class = get_class(class_name);
}

void
RDF_Connector::initialize(std::string serverID, std::string repoID) {
	std::cout << "Initializing RDF connection to " << serverID << " with repo name " << repoID << std::endl;

	jstring sid = create_string(serverID);
	jstring rid = create_string(repoID);

	jmethodID mid = get_static_mid(main_class, "initialize", "(Ljava/lang/String;Ljava/lang/String;)V");
	call_static_void_method(main_class, mid, sid, rid);

	mid = get_static_mid(main_class, "clearRepo", "()V");
	call_static_void_method(main_class, mid);
}

void 
RDF_Connector::add_scene (int scene_index) {
	jmethodID mid = get_static_mid(main_class, "addScene", "(I)V");
	call_static_void_method(main_class, mid, scene_index);
}

void 
RDF_Connector::add_cluster (int scene_index, int cluster_index) {
	jmethodID mid = get_static_mid(main_class, "addCluster", "(II)V");
	call_static_void_method(main_class, mid, scene_index, cluster_index);
}

void 
RDF_Connector::add_current_position (int scene_index, std::string position) {
	jstring pos = create_string(position);
	jmethodID mid = get_static_mid(main_class, "addCurrentPosition", "(ILjava/lang/String;)V");
	call_static_void_method(main_class, mid, scene_index, pos);
}

void 
RDF_Connector::add_next_position (int scene_index, int cluster_index, std::string position)  {
	jstring pos = create_string(position);
	jmethodID mid = get_static_mid(main_class, "addNextPosition", "(IILjava/lang/String;)V");
	call_static_void_method(main_class, mid, scene_index, cluster_index, pos);
}

void 
RDF_Connector::add_model (int scene_index, int cluster_index, std::string name, double score, std::string pose) {
	jstring n = create_string(name);
	jstring p = create_string(pose);
	jmethodID mid = get_static_mid(main_class, "addModel", "(IILjava/lang/String;DLjava/lang/String;)V");
	call_static_void_method(main_class, mid, scene_index, cluster_index, n, score, p);
}

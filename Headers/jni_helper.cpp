#include "jni_helper.hpp"

JNI_Helper::JNI_Helper() {

}

/**
	Initialize the class_path to the VM
	@param class_path The class path
*/
JNI_Helper::JNI_Helper(std::string class_path) {
	initialize(class_path);
}

void
JNI_Helper::initialize(std::string class_path) {
	std::string jvm_command = "-Djava.class.path=" + class_path;

	const int kNumOptions = 1;
	JavaVMOption options[kNumOptions] = {
		{ const_cast<char*>(jvm_command.c_str()), NULL }
	};

	JavaVMInitArgs vm_args;
	vm_args.version = JNI_VERSION_1_6;
	vm_args.nOptions = kNumOptions;
	vm_args.options = options;

	int res = JNI_CreateJavaVM(&jvm, reinterpret_cast<void**>(&env), &vm_args);
	if (res != JNI_OK) {
		throw std::runtime_error("JNI_Helper::JNI_Helper : JNI_CreateJavaVM failed!");
	} else {
		std::cerr << "JVM created!" << std::endl;
	}
}

JNI_Helper::~JNI_Helper() {
	jvm->DestroyJavaVM();

	std::cout << "JVM destroyed!" << std::endl;
}

/*
	Get a class from a program in the JVM
	@param class_name The name of the class
	@return The jclass object
*/
jclass 
JNI_Helper::get_class(std::string class_name) {
	jclass c = env->FindClass(class_name.c_str());
	if (c == NULL) {
		throw std::runtime_error("get_class : Class \"" + class_name + "\" not found!");
	}
}

/*
	Get a method ID from a class in the JVM
	@param class_j The jclass object to find the method in
	@param method_name The name of the method
	@param signature The method signature
	@return The jmethodID object
*/
jmethodID
JNI_Helper::get_static_mid(jclass class_j, std::string method_name, std::string signature) {
	jmethodID mid = env->GetStaticMethodID(class_j, method_name.c_str(), signature.c_str());
	if (mid == NULL) {
		throw std::runtime_error("get_static_mid : Method \"" + method_name + "\" not found!");
	}
}

jstring 
JNI_Helper::create_string(std::string str) {
	jstring jstr = env->NewStringUTF(str.c_str());
	return jstr;
}

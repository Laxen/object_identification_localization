#ifndef JNI_HELPER_H
#define JNI_HELPER_H

#include <jni.h>
#include <iostream>
#include <stdexcept>

class JNI_Helper {
	private:
		JNIEnv* env;
		JavaVM* jvm;
	public:
		JNI_Helper(std::string class_path);
		~JNI_Helper();

		/*
			Get a class from a program in the JVM
			@param class_name The name of the class
			@return The jclass object
		*/
		jclass 
		get_class(std::string class_name);

		/*
			Get a method ID from a class in the JVM
			@param class_j The jclass object to find the method in
			@param method_name The name of the method
			@param signature The method signature
			@return The jmethodID object
		*/
		jmethodID
		get_static_mid(jclass class_j, std::string method_name, std::string signature);

		/*
			Calls a static double method in a class in the JVM
			@param class_j The class where the method is
			@param mid The method ID
			@param args All arguments that should be passed to the function
			@return The double as a jdouble object
		*/
		template<typename... Types>
		jdouble
		call_static_double_method(jclass class_j, jmethodID mid, Types ...args) {
			jdouble val = env->CallStaticDoubleMethod(class_j, mid, args...);
			return val;
		}

		/*
			Calls a static void method in a class in the JVM
			@param class_j The class where the method is
			@param mid The method ID
			@param args All arguments that should be passed to the function
		*/
		template<typename... Types>
		void
		call_static_void_method(jclass class_j, jmethodID mid, Types ...args) {
			env->CallStaticDoubleMethod(class_j, mid, args...);
		}

		jstring 
		create_string(std::string str);
};

#endif

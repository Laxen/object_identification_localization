#ifndef ROBOT_DATA_FETCHER_H
#define ROBOT_DATA_FETCHER_H

#include <iostream>
#include <string>
#include <curl/curl.h>
#include <fstream>
#include <vector>
#include <stdlib.h>

class Robot_Data_Fetcher {
	private:
		std::string username;
		std::string password;
		std::string curl_url;

		static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);

		/**
			Reads data from class variable
			@param readBuffer The string to read data from
			@param variable The variable to read
			@return The string of data for the variable
		*/
		std::string find_data(std::string readBuffer, std::string variable);

	public:
		/**
			Fetches xyz and quaternion data, using authentication from set_auth and URL from set_url
			@param save_path Path where the data should be saved
		*/
		void fetch_data(std::string save_path);

		/**
			Set the authentication details for the robot web service
			@param user The username
			@param pass The password
		*/
		void
		set_auth_details(std::string user, std::string password);

		/**
			Set the URL to the robot web service (URL to hand which position should be read)
			@param url The url
		*/
		void
		set_url(std::string url);
};

#endif

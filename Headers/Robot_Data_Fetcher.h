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

		std::string find_data(std::string readBuffer, std::string variable);

	public:
		void fetch_data(std::string save_path);

		std::vector<double>
		fetch_data_vector(std::string save_path);

		void
		set_auth_details(std::string user, std::string password);

		void
		set_url(std::string url);
};

#endif

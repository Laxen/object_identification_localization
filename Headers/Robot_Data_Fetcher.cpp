#include "Robot_Data_Fetcher.h"

size_t
Robot_Data_Fetcher::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
	((std::string*)userp)->append((char*)contents, size * nmemb);
	return size * nmemb;
}

std::string
Robot_Data_Fetcher::find_data(std::string readBuffer, std::string variable) {
	int index_start = (int)readBuffer.find ("<span class=\"" + variable + "\"");
	int index_x = (int)readBuffer.find (">", index_start);
	int index_y = (int)readBuffer.find ("<", index_x);

	std::string x_str = readBuffer.substr (index_x+1, index_y - (index_x+1));
	return x_str;
}


void
Robot_Data_Fetcher::set_auth_details(std::string user, std::string pass) {
	username = user;
	password = pass;
}

void
Robot_Data_Fetcher::set_url(std::string url) {
	curl_url = url;
}

void
Robot_Data_Fetcher::fetch_data(std::string save_path) {
	CURL *curl;
	CURLcode res;
	std::string readBuffer;

	curl = curl_easy_init();
	if(curl) {
		curl_easy_setopt (curl, CURLOPT_HTTPAUTH, (long)CURLAUTH_ANY);
		//curl_easy_setopt (curl, CURLOPT_USERPWD, "Default User:robotics");
		//curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.3/rw/rapid/tasks/T_ROB_L/motion?resource=robtarget&tool0&wobj=wobj0");
		curl_easy_setopt (curl, CURLOPT_USERPWD, username + ":" + password);
		curl_easy_setopt(curl, CURLOPT_URL, curl_url);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
		res = curl_easy_perform(curl);
		curl_easy_cleanup(curl);

		std::ofstream ofs;
		ofs.open(save_path.c_str());
		ofs << find_data(readBuffer, "x") << ",";
		ofs << find_data(readBuffer, "y") << ",";
		ofs << find_data(readBuffer, "z") << ",";
		ofs << find_data(readBuffer, "q1") << ",";
		ofs << find_data(readBuffer, "q2") << ",";
		ofs << find_data(readBuffer, "q3") << ",";
		ofs << find_data(readBuffer, "q4") << ",";
		ofs.close();
	}
}

std::vector<double>
Robot_Data_Fetcher::fetch_data_vector(std::string save_path) {
	CURL *curl;
	CURLcode res;
	std::string readBuffer;

	curl = curl_easy_init();
	if(curl) {
		curl_easy_setopt (curl, CURLOPT_HTTPAUTH, (long)CURLAUTH_ANY);
		curl_easy_setopt (curl, CURLOPT_USERPWD, "Default User:robotics");
		curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.3/rw/rapid/tasks/T_ROB_L/motion?resource=robtarget&tool0&wobj=wobj0");
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
		res = curl_easy_perform(curl);
		curl_easy_cleanup(curl);

		std::vector<double> data;
		data.push_back(atof(find_data(readBuffer, "x").c_str()));
		data.push_back(atof(find_data(readBuffer, "y").c_str()));
		data.push_back(atof(find_data(readBuffer, "z").c_str()));
		data.push_back(atof(find_data(readBuffer, "q1").c_str()));
		data.push_back(atof(find_data(readBuffer, "q2").c_str()));
		data.push_back(atof(find_data(readBuffer, "q3").c_str()));
		data.push_back(atof(find_data(readBuffer, "q4").c_str()));


		std::ofstream ofs;
		ofs.open(save_path.c_str());
		ofs << find_data(readBuffer, "x") << ",";
		ofs << find_data(readBuffer, "y") << ",";
		ofs << find_data(readBuffer, "z") << ",";
		ofs << find_data(readBuffer, "q1") << ",";
		ofs << find_data(readBuffer, "q2") << ",";
		ofs << find_data(readBuffer, "q3") << ",";
		ofs << find_data(readBuffer, "q4") << ",";
		ofs.close();

		return data;
	}
}

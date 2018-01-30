
#include "normal_utility.h"

/**
  Saturate the value so that it does not go below 90
  @param value The value to be saturated
  @return The saturated value
*/
float 
Normal_Utility::saturate (float value)
{
	if (value < 90)
	{
		value = 90;
	}
	
	return value;
}

/**
  Computes the normal utilities. Each point and is given an angle between its normal and the viewdirection. The final utility value of the view is then the average of all angles in the cloud. 
  @param views The view point clouds
  @return The vector containing all the normal-utilities for all views
*/
std::vector<float>
Normal_Utility::compute_normal_utilities (std::vector<PointCloud_N::Ptr> views)
{
	std::vector<float> normal_utilities;
	
	float pi_const = atan(1)*4;
	for (int i = 0; i < views.size(); i++)
	{
		std::vector<float> angle_vec;
		for (int j = 0; j < views[i]->points.size(); j++)
		{
			// Compute the angle between the normal and the viewdirection 
			float angle = std::abs (acos (views[i]->points[j].normal_z) * (180 / pi_const));
			angle_vec.push_back (saturate (angle));
		}
		
		// Compute the average angle for the cloud
		float avg_angle = std::accumulate( angle_vec.begin(), angle_vec.end(), 0.0) / angle_vec.size();
		normal_utilities.push_back (avg_angle);
	}
	
	return normal_utilities;
}

/**
  Computes the normal utilities. A bad normal is considered a normal thats almost perpendicular to the viewing direction.
  @param views The view point clouds
  @param viewpoint The viewpoint origin
  @param bad_normal_thr Threshold determining if a normal is bad. 90 degrees => normal perpendicular to viewpoint origin. if (angle between normal vector and viewpoint vector > 90 + bad_normal_thr) then the normal is considered to be bad. 
  @return The vector containing all the normal-utilities for all views
*/
std::vector<float>
Normal_Utility::compute_normal_utilities_2 (std::vector<PointCloud_N::Ptr> views, PointT viewpoint, float bad_normal_thr)
{
	std::vector<float> normal_utilities;
	
	/*
	float pi_const = atan(1)*4;
	for (int i = 0; i < views.size(); i++)
	{
		int nbr_of_bad_normals = 0;
		for (int j = 0; j < views[i]->points.size(); j++)
		{		
			// Compute the angle between the normal and the viewdirection 
			float angle = std::abs (acos (views[i]->points[j].normal_z) * (180 / pi_const));
			if (angle < 110)
			{
				nbr_of_bad_normals++;
			}
		}
		
		normal_utilities.push_back (nbr_of_bad_normals);
	}
	*/
	
	float pi_const = atan(1)*4;
	for (int i = 0; i < views.size(); i++)
	{
		int nbr_of_bad_normals = 0;
		for (int j = 0; j < views[i]->points.size(); j++)
		{		
			// Compute the angle between the normal and the viewdirection 
			Eigen::Vector3f n;
			n << views[i]->points[j].normal_x, 
		   		 views[i]->points[j].normal_y, 
		   		 views[i]->points[j].normal_z;
			Eigen::Vector3f v_p;
			v_p << views[i]->points[j].x - viewpoint.x,
				   views[i]->points[j].y - viewpoint.y,
				   views[i]->points[j].z - viewpoint.z;
			n.normalize ();
			v_p.normalize ();
			float angle = acos (n.dot(v_p)) * (180 / pi_const);
			if (angle < (90 + bad_normal_thr))
			{
				nbr_of_bad_normals++;
			}
		}
		
		// Compute the fraction of bad normals for the whole point cloud
		float frac_bad_normals = 1 - nbr_of_bad_normals / (float)views[i]->points.size();
		normal_utilities.push_back (frac_bad_normals);
	}
	
	//
	// Normalize utilities
	//
	
	std::vector<float> temp = normal_utilities;
	std::sort(temp.begin(), temp.end());
	
	float max_value = temp.back();
	float min_value = temp.front();
	for (int i = 0; i < normal_utilities.size(); i++)
	{
		normal_utilities[i] = (normal_utilities[i] - min_value) / (max_value - min_value);
	}
	
	/*
	for (int i = 0; i < normal_utilities.size(); i++)
	{
		printf ("%3.3f\n", normal_utilities[i]);
	}
	*/
	
	return normal_utilities;
}

/**
  Normalizes the vector between 180 and 90
  @param vec The vector to be normalized
*/
void
Normal_Utility::normalize (std::vector<float> &vec)
{
	/*
	// Sort vec
	std::vector<float> temp = vec;
	std::sort(temp.begin(), temp.end());
	
	float max_value = temp.back();
	float min_value = temp.front();
	for (int i = 0; i < vec.size(); i++)
	{
		vec[i] = (vec[i] - min_value) / (max_value - min_value);
	}
	*/
	
	float max_value = 180;
	float min_value = 90;
	for (int i = 0; i < vec.size(); i++)
	{
		vec[i] = (vec[i] - min_value) / (max_value - min_value);
	}
}

/**
  Computes new utilities according to some function. The new utilities are still within [0,1] 
  @param utilities The normal-utility vector
*/
void
Normal_Utility::new_utilities (std::vector<float> &utilities)
{
	for (int i = 0; i < utilities.size(); i++)
	{
		/*
		// 2x^2
		float temp = 2 * pow (utilities[i],2);
		if (temp > 1)
		{
			temp = 1;
		}
		utilities[i] = temp;
		*/
		
		/*
		// exp(x)-1
		float temp = exp (1.2*utilities[i]) - 1;
		if (temp > 1)
		{
			temp = 1;
		}
		utilities[i] = temp;
		*/
		
		/*
		// 1-(x-1)^4
		utilities[i] = 1 - pow ((utilities[i] - 1),4);
		*/
		
		//*
		// Saturated linear function
		float temp = utilities[i];
		if (temp > 0.5)
		{
			temp = 1;
		}
		else if (temp < 0.3)
		{
			temp = 0;
		}
		else
		{
			temp = 5 * temp - 1.5;
		}
		
		utilities[i] = temp;
		//*/
	}
}

/**
  Generates normal-utilities for the given rendered views of a model
  @param views The view point clouds
  @param viewpoint The viewpoint origin
  @param bad_normal_thr Threshold determining if a normal is bad. 90 degrees => normal perpendicular to viewpoint origin. if (angle between normal vector and viewpoint vector > 90 + bad_normal_thr) then the normal is considered to be bad. 
  @param mode The mode = 1 computes the number of bad normals for each view and mode != 1 computes the average normal angel for each view 
  @return The normal-utility vector
*/
std::vector<float>
Normal_Utility::generate_normal_utilities (std::vector<PointCloud_N::Ptr> views, PointT viewpoint, float bad_normal_thr, int mode)
{	
	std::vector<float> normal_utilities;
	if (mode == 1)
	{
		// Compute normal utility for each view according to the number of bad normals for each view
		normal_utilities = compute_normal_utilities_2 (views, viewpoint, bad_normal_thr);
		
		/*
		for (int i = 0; i < normal_utilities.size(); i++)
		{
			printf ("%3.3f\n", normal_utilities[i]);
		}
		*/
	}
	else
	{
		// Compute normal utility for each view according to the average normal angle
		normal_utilities = compute_normal_utilities (views);
	
		/*
		for (int i = 0; i < normal_utilities.size(); i++)
		{
			printf ("%3.3f\n", normal_utilities[i]);
		}
		*/
	
		// Normalize normal utilities
		normalize (normal_utilities);
	
		/*
		for (int i = 0; i < normal_utilities.size(); i++)
		{
			printf ("%3.3f\n", normal_utilities[i]);
		}
		*/
	
		// Compute new utility values according to some function
		new_utilities (normal_utilities);
	}
	
	return normal_utilities;
}

/**
  Saves the normal-utilities
  @param normal_utilities The utility vector
  @param model_name The name of the model
*/
void
Normal_Utility::save_normal_utilities (std::string model_name, std::vector<float> normal_utilities)
{
	Access_Model_Data amd;
	boost::filesystem::path p = amd.path_to_model_in_model_data (model_name);
	p /= "normal_utilities.csv";
	
	std::ofstream utilities_file;
	utilities_file.open (p.string().c_str());
	
	std::vector<std::string> string_vec;
	for (int i = 0; i < normal_utilities.size(); i++)
	{
		std::ostringstream ss;
		ss << i << ",";
		ss << normal_utilities[i] << ",";
		string_vec.push_back (ss.str ());
	}
	
	for (int i = 0; i < string_vec.size(); i++)
	{
		utilities_file << string_vec[i] << "\n";
	}
	
	utilities_file.close();
	
}

















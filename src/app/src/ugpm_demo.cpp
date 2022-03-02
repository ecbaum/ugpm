/**
 *  Author: Cedric LE GENTIL
 *
 *  Copyright 2021 Cedric LE GENTIL
 *
 *  This is a simple example of how to generate the UGPMs
 *  
 *  Disclaimer:
 *  This code is not optimised neither for performance neither for maintainability.
 *  There might still be errors in the code, logic or procedure. Every feedback is welcomed.
 *
 *  For any further question or if you see any problem in that code
 *  le.gentil.cedric@gmail.com
 **/


#include <iostream>
#include <string>
#include <stdio.h>
#include <fstream>
#include <iomanip> 
#include <boost/program_options.hpp>
#include "sensor_input/imu_simulator.h"
#include "imu_preintegration/preintegration.h"
#include "common/random.h"
#include "common/utils.h"

std::string get_path(std::string &str, int N){

    int start_of_path;
    std::string path;
    for (int i = 0; i < N; i++) {
        if(int(str[i]) == 58){
            start_of_path = i+1;
        }
    }
    path = str.substr(start_of_path, N);
    path.erase(std::remove_if(path.begin(), path.end(), [](unsigned char x) { return std::isspace(x); }), path.end());
    
    return path;
}

std::vector<std::string> readYAML()
{
    /* This is a primitive yaml-reader custom build for the expected yaml-file*/

    std::ifstream f;

    f.open ("../config.yaml");   /* open file with filename as argument */
    if (! f.is_open()) {    /* validate file open for reading */
        std::cerr << "error: file open failed.\n";
    }

    std::vector<std::string> expected_strings = {"imu_raw:", "sampling_times:", "save_path"};
    std::vector<std::string> paths;

    int expected_amount = expected_strings.size();
    int retrieved_amount = 0;

    bool in_ugpm = false;
    std::string line;

    while (std::getline (f, line)) {        /* read each line */
        if(line.find("ugpm:") != std::string::npos){
            in_ugpm = true;
        }
        if(in_ugpm){
            for(std::string str : expected_strings){
                if ( line.find(str) != std::string::npos && retrieved_amount<expected_amount) {
                    int N = line.length();
                    std::string path = get_path(line,N);
                    paths.push_back(path);
                    retrieved_amount++;
                }
            }
        }
    }
    return paths;
}



std::vector<std::vector<double>> readCSV(std::string path)
{
    std::ifstream f;
    std::cout << "reading csv file \n";

    f.open (path);   /* open file with filename as argument */
    if (! f.is_open()) {    /* validate file open for reading */
        std::cerr << "error: file open failed.\n";
    }

    std::string line, val;                  /* string for line & value */
    std::vector<std::vector<double>> array;    /* vector of vector<int>  */

    while (std::getline (f, line)) {        /* read each line */
        std::vector<double> v;                 /* row vector v */
        std::stringstream s (line);         /* stringstream line */
        while (getline (s, val, ','))       /* get each value (',' delimited) */
            v.push_back (std::stod (val));  /* add to row vector */
        array.push_back (v);                /* add row vector to array */
    }

    return array;
}

void saveCSV(std::string path, std::vector<std::vector<double>> *array, std::string seperator, int precision, std::string header){

    std::ofstream myfile (path);
    std::cout << "Saving csv to " << path << "\n"; 
    
    bool write_header = false;

    if (myfile.is_open()){
        if(!header.empty()){
            myfile << header << "\n";
        }
        for (const auto& inner : *array) {
            for (const auto& item : inner) {

                myfile << std::fixed << std::setprecision(precision) << item << seperator << " ";
            }
            myfile << "\n";  
        }
        myfile.close();
    }else{
        std::cout << "Unable to open file";} 
}

celib::ImuData convertArray(std::vector<std::vector<double>> *array){
    
    std::cout << "converting csv array to ImuData format: \n";
    int i;
    double t, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;

    celib::ImuData imu_data;

    for (auto& row : *array) {               /* iterate over rows */
        celib::ImuSample acc, gyr;
        i = 0;
        for (auto& val : row){               /* iterate over vals */

            switch(i){
                case 0: t = val;
                case 1: acc_x = val;
                case 2: acc_y = val;
                case 3: acc_z = val;
                case 4: gyr_x = val;
                case 5: gyr_y = val;
                case 6: gyr_z = val; 
            }
            i++;        
        }

        double acc_data[3] = {acc_x, acc_y, acc_z};
        double gyr_data[3] = {gyr_x, gyr_y, gyr_z};
        acc.t = t;
        gyr.t = t;
        memcpy(acc.data, acc_data, sizeof(acc_data));
        memcpy(gyr.data, gyr_data, sizeof(gyr_data));

        imu_data.acc.push_back(acc);
        imu_data.gyr.push_back(gyr);
        imu_data.acc_var = 1.25e-7;
        imu_data.gyr_var = 2.0e-4;
        
    }
    return imu_data;
}

std::vector<std::vector<double>> integrate_between_samples(celib::ImuData imu_data, std::vector<std::vector<double>> sample_time_array, celib::PreintOption preint_opt){
    /*
        Format is

        t_i delta_t a_x a_y a_z w_x w_y w_z

        t_i is current time
        delta_t is t_i - t_i-1
        a_x, a_y, a_z is defined as v_i - v_i-1
        w_x, w_y, w_z is defined as angle_i - angle_i-1

    */
    double t_iter, t_iter_prev;

    t_iter_prev = sample_time_array[0][0];

    int a = 0;
    int N = sample_time_array.size();

    std::vector<std::vector<double>> data_array;

    for(int i = 0; i < N; i++){

        std::vector<double> data_iter;

        t_iter = sample_time_array[i][0];

        celib::PreintPrior prior;
    
        double start_t = t_iter_prev;

        std::vector<std::vector<double> > t;
        std::vector<double> tmp1_ = {t_iter};
        t.push_back(tmp1_);

        
        celib::StopWatch stop_watch;
        stop_watch.start();
        celib::ImuPreintegration preint(imu_data, start_t, t, preint_opt, prior);
        stop_watch.stop();
        stop_watch.print();

        celib::PreintMeas preint_meas = preint.get(0,0);
        
        // This conversion between R and euler angles may be a possible source of error
        Eigen::Vector3d euler_t = preint_meas.delta_R.eulerAngles(2, 1, 0);

        
        data_iter.push_back(t_iter);
        data_iter.push_back(t_iter - t_iter_prev);
        data_iter.push_back(preint_meas.delta_v.coeff(0,0));
        data_iter.push_back(preint_meas.delta_v.coeff(0,1));
        data_iter.push_back(preint_meas.delta_v.coeff(0,2));
        data_iter.push_back(euler_t[0]);
        data_iter.push_back(euler_t[1]);
        data_iter.push_back(euler_t[2]);

        data_array.push_back(data_iter);
        t_iter_prev = t_iter;
    }
    return data_array;
}

void pre_integrate_between_frames(celib::PreintOption preint_opt){
    std::vector<std::string> paths = readYAML();
    std::vector<std::vector<double>> imu_array, sample_time_array, data_array;

    imu_array = readCSV(paths[0]);
    sample_time_array = readCSV(paths[1]);

    celib::ImuData imu_data = convertArray(&imu_array);

    data_array = integrate_between_samples(imu_data, sample_time_array, preint_opt);

    saveCSV(paths[2], &data_array, "", 9, "Time deltatime a_x a_y a_z w_x w_y w_z");
}

int main(int argc, char* argv[]){

    celib::PreintOption preint_opt;
    bool test_jacobians;


    // Program options
    boost::program_options::options_description opt_description("Allowed options");
    opt_description.add_options()
        ("help,h", "Produce help message")
        ("method,m", boost::program_options::value< std::string >(), "LPM | GPM | UGPM(default)")
        ("length,l", boost::program_options::value< double >(), "Length simulated integration (default = 2.0)")
        ("train,t", boost::program_options::bool_switch(&preint_opt.train_gpm), "Flag to activate training")
        ("jacobian,j", boost::program_options::bool_switch(&test_jacobians), "Flag to activate the testing of the Jacobians")
        ("nb_inference,n", boost::program_options::value< int >(), "Nb of inferrences (to simulate lidar nb of inferences), Default = 1")
        ("quantum,q", boost::program_options::value< double >(), "Time quantum for piece-wise integration (useful for long integration intervals, -1 to deactivate (default = 0.2)")
        ;

    boost::program_options::variables_map var_map;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt_description), var_map);
    boost::program_options::notify(var_map);    

    if(var_map.count("help")) {
        std::cout << opt_description << std::endl;
        return 1;
    }

    preint_opt.type = celib::UGPM;
    if(var_map.count("method"))
    {
        std::string type = var_map["method"].as<std::string>();
        std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c){ return std::tolower(c); });
        if(type == "lpm") preint_opt.type = celib::LPM;
        else if(type == "gpm") preint_opt.type = celib::GPM;
        else if(type == "ugpm") preint_opt.type = celib::UGPM;
        else
        {
            std::cout << "ERROR: The type of preintegration method is unknown, program stopping now" << std::endl;
            return 1;
        }
    }
    std::string to_print;
    switch (preint_opt.type)
    {
    case celib::LPM:
        to_print = "LPM";
        break;
    
    case celib::GPM:
        to_print = "GPM";
        break;
    
    case celib::UGPM:
        to_print = "UGPM";
        break;
    
    default:
        break;
    }
    std::cout << "Preintegration demonstration with " << to_print << std::endl;

    double integration_length = 2;
    if(var_map.count("length"))
    {
        integration_length = var_map["length"].as<double>();
    }

    int nb_infer = 0;
    if(var_map.count("nb_inference"))
    {
        nb_infer = var_map["nb_inference"].as<int>() - 1;
    }

    preint_opt.quantum = 0.02;
    if(var_map.count("quantum"))
    {
        preint_opt.quantum = var_map["quantum"].as<double>();
    }

    preint_opt.min_freq = 1000;

    pre_integrate_between_frames(preint_opt);

  /*
    std::vector<double> error = imu_sim.testPreint(start_t, end_t, preint.get(0,0));

    std::cout << "Preintegration errors over window of " << preint_meas.dt << " seconds:" << std::endl;
    std::cout << "  Rotation [deg] = " << error[0]*180.0/M_PI << std::endl;
    std::cout << "  Velocity [m/s] = " << error[1] << std::endl;
    std::cout << "  Position [m]   = " << error[2] << std::endl;
    std::cout << std::endl;
    std::cout << "Covariance" << std::endl << preint_meas.cov << std::endl;
    std::cout << std::endl;

    
    */
    
    /*
    if(test_jacobians)
    {
        double num_quantum = 0.001;

        // Compute numerical preintegrated measurements to compare
        celib::PreintMeas preint_num_jacobian;
        for(int i = 0; i < 3; ++i)
        {
            auto data_temp_bw = data;
            auto data_temp_bf = data;
            for(int j = 0; j < data.gyr.size(); ++j) data_temp_bw.gyr[j].data[i] += num_quantum;
            for(int j = 0; j < data.acc.size(); ++j) data_temp_bf.acc[j].data[i] += num_quantum;

            celib::ImuPreintegration preint_bw(data_temp_bw, start_t, t, preint_opt, prior);
            auto temp_preint = preint_bw.get(0,0);

            preint_num_jacobian.d_delta_R_d_bw.col(i) = celib::LogMap(preint_meas.delta_R.transpose()*temp_preint.delta_R)/num_quantum;
            preint_num_jacobian.d_delta_v_d_bw.col(i) = (temp_preint.delta_v - preint_meas.delta_v)/num_quantum;
            preint_num_jacobian.d_delta_p_d_bw.col(i) = (temp_preint.delta_p - preint_meas.delta_p)/num_quantum;

            celib::ImuPreintegration preint_bf(data_temp_bf, start_t, t, preint_opt, prior);
            temp_preint = preint_bf.get(0,0);

            preint_num_jacobian.d_delta_v_d_bf.col(i) = (temp_preint.delta_v - preint_meas.delta_v)/num_quantum;
            preint_num_jacobian.d_delta_p_d_bf.col(i) = (temp_preint.delta_p - preint_meas.delta_p)/num_quantum;
        }
        auto data_temp_dt = data;
        for(int j = 0; j < data.gyr.size(); ++j) data_temp_dt.gyr[j].t -= num_quantum;
        for(int j = 0; j < data.acc.size(); ++j) data_temp_dt.acc[j].t -= num_quantum;

        celib::ImuPreintegration preint_dt(data_temp_dt, start_t, t, preint_opt, prior);
        auto temp_preint = preint_dt.get(0,0);
        preint_num_jacobian.d_delta_R_d_t = celib::LogMap(preint_meas.delta_R.transpose()*temp_preint.delta_R)/num_quantum;
        preint_num_jacobian.d_delta_v_d_t = (temp_preint.delta_v - preint_meas.delta_v)/num_quantum;
        preint_num_jacobian.d_delta_p_d_t = (temp_preint.delta_p - preint_meas.delta_p)/num_quantum;


        // Print the jacobians 2 by two to "easily" read the difference
        std::cout << std::endl;
        std::cout << "Test Jacobians" << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta R / gyr bias" << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_R_d_bw << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_R_d_bw << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta R / dt " << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_R_d_t.transpose() << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_R_d_t.transpose() << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta v / gyr bias" << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_v_d_bw << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_v_d_bw << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta v / acc bias" << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_v_d_bf << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_v_d_bf << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta v / dt " << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_v_d_t.transpose() << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_v_d_t.transpose() << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta p / gyr bias" << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_p_d_bw << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_p_d_bw << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta p / acc bias" << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_p_d_bf << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_p_d_bf << std::endl;
        std::cout << std::endl;
        std::cout << "Jacobian Delta p / dt " << std::endl;
        std::cout << "Computed" << std::endl;
        std::cout << preint_meas.d_delta_p_d_t.transpose() << std::endl;
        std::cout << "Numerical" << std::endl;
        std::cout << preint_num_jacobian.d_delta_p_d_t.transpose() << std::endl;
        std::cout << std::endl;
    }
    */


    return 0;
}

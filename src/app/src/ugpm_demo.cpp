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

#include <boost/program_options.hpp>
#include "sensor_input/imu_simulator.h"
#include "imu_preintegration/preintegration.h"
#include "common/random.h"
#include "common/utils.h"


std::vector<std::vector<double>> readCSV()
{
    std::ifstream f;

    f.open ("../test.txt");   /* open file with filename as argument */
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

void convertArray(std::vector<std::vector<double>> *array){
    
    std::cout << "iterating over loaded csv file: \n";
    for (auto& row : *array) {               /* iterate over rows */
        for (auto& val : row)               /* iterate over vals */
            std::cout << val << "  ";       /* output value      */
        std::cout << "\n";                  /* tidy up with '\n' */
    }

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

    preint_opt.quantum = 0.2;
    if(var_map.count("quantum"))
    {
        preint_opt.quantum = var_map["quantum"].as<double>();
    }






    // Create an IMU simulator
    celib::ImuSimulatorOption sim_opt;
    sim_opt.acc_std = 0.02;
    sim_opt.gyr_std = 0.002;
    celib::ImuSimulator imu_sim(sim_opt);

    // Create some fake data
    double overlap = 0.15;
    celib::RandomGenerator rand_gen;
    double start_t = rand_gen.randUniform(overlap,sim_opt.dataset_length - integration_length - overlap);
    double end_t = start_t + integration_length;
    celib::ImuData data = imu_sim.get(start_t-overlap, end_t+overlap);

    data.print();

    // Create a preintegration object
    preint_opt.min_freq = 1000;
    celib::PreintPrior prior;
    std::vector<std::vector<double> > t;
    std::vector<double> temp_t;
    temp_t.push_back(end_t);
    t.push_back(temp_t);
    std::vector<double> temp_t_2;
    for(int i = 0; i < nb_infer; ++i)
    {
        double ratio = i/(double)(nb_infer);
        temp_t_2.push_back(start_t*(1-ratio) + end_t*ratio);
    }
    t.push_back(temp_t_2);

    // Test some data
    celib::StopWatch stop_watch;
    stop_watch.start();
    celib::ImuPreintegration preint(data, start_t, t, preint_opt, prior);
    stop_watch.stop();
    stop_watch.print();

    celib::PreintMeas preint_meas = preint.get(0,0);

    std::vector<double> error = imu_sim.testPreint(start_t, end_t, preint.get(0,0));

    std::cout << "Preintegration errors over window of " << preint_meas.dt << " seconds:" << std::endl;
    std::cout << "  Rotation [deg] = " << error[0]*180.0/M_PI << std::endl;
    std::cout << "  Velocity [m/s] = " << error[1] << std::endl;
    std::cout << "  Position [m]   = " << error[2] << std::endl;
    std::cout << std::endl;
    std::cout << "Covariance" << std::endl << preint_meas.cov << std::endl;
    std::cout << std::endl;

    preint_meas.print();

    std::vector<std::vector<double>> array = readCSV();

    convertArray(&array);

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



    return 0;
}

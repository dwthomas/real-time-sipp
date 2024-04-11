#include <iostream>
#include <filesystem>
#include <ostream>
#include <boost/program_options.hpp>
#include "hybrid.hpp"
#include "newatsippgraph.hpp"
#include "randomsippgraph.hpp"
#include "sipp.hpp"
#include "augmentedsipp.hpp"
#include "rtasipp.hpp"
#include "plrtosipp.hpp"
#include "plrtosipphonly.hpp"
#include "sippgraph.hpp"
#include "structs.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    try{
        // Declare options
        po::options_description desc("Allowed options");
        desc.add_options()
        ("help,h", "produce help message")
        ("startx,x", po::value<int>(), "x position of agent starting location")
        ("starty,y", po::value<int>(), "y position of agent starting location")
        ("goalx,X", po::value<int>(), "x position of goal location")
        ("goaly,Y", po::value<int>(), "y position of goal location")
        ("map,m", po::value<std::filesystem::path>(),"static map file")
        ("search,s", po::value<std::string>(), "Search algorithm to use")
        ("startTime,t", po::value<double>()->default_value(0.0), "Start Time of search.")
        ("until,u", po::value<double>()->default_value(10.0), "Max time for random obstacles.")
        ("occupancy,o", po::value<double>()->default_value(0.0), "Occupancy of random obstacles")
        ("minDuration", po::value<double>()->default_value(1.0), "Min interval duration")
        ("maxDuration", po::value<double>()->default_value(10.0), "Max interval duration")
        ("budget,b", po::value<long>()->default_value(1), "Search budget in num of expansions")
        ("seed", po::value<long>()->default_value(0), "Random seed")
        ;
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);   
        // Parse options
        if (vm.count("help")){
            std::cout << desc << std::endl;
        }
        else if(vm.count("map") && std::filesystem::is_regular_file(vm["map"].as<std::filesystem::path>())){
            // read map
            Location source_loc(vm["startx"].as<int>(), vm["starty"].as<int>());
            Location goal_loc(vm["goalx"].as<int>(), vm["goaly"].as<int>());
            //std::cerr << "Generating SIPP graph...";
            Map m(vm["map"].as<std::filesystem::path>().string());
            double until = vm["until"].as<double>();
            double occupancy = vm["occupancy"].as<double>();
            double min_duration = vm["minDuration"].as<double>();
            double max_duration = vm["maxDuration"].as<double>();
            SippGraph<Location> g = make_random_sipp_graph(m, until, occupancy, min_duration, max_duration, source_loc, goal_loc, vm["seed"].as<long>());

            AtSippGraph<Location> atg(&g);
            double start_time = vm["startTime"].as<double>();
            const SIPPState<Location> * source = find_earliest(g, source_loc, start_time);
            if(vm["search"].as<std::string>() == "sipp"){
                MetaData m;
                auto res = sipp::search(g, source, goal_loc, m, start_time);
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
            else if(vm["search"].as<std::string>() == "asipp"){
                MetaData m;
                auto res = asipp::search(atg, source, goal_loc, m, start_time);
                for(auto n: res.first){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
            else if(vm["search"].as<std::string>() == "rtas"){
                MetaData m;
                long budget = vm["budget"].as<long>();
                auto res = rtasipp::search(atg, source, goal_loc, m, start_time, budget);
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
            else if(vm["search"].as<std::string>() == "maxatfs"){
                MetaData m;
                long budget = vm["budget"].as<long>();
                auto res = plrtosipp::search(atg, source, goal_loc, m, budget, start_time);
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
            else if(vm["search"].as<std::string>() == "plrts"){
                MetaData m;
                long budget = vm["budget"].as<long>();
                auto res = plrtosipphonly::search(atg, source, goal_loc, m, budget, start_time);
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
            else if(vm["search"].as<std::string>() == "medatfs"){
                MetaData m;
                long budget = vm["budget"].as<long>();
                auto res = hybrid::search(atg, source, goal_loc, m, budget, start_time);
                for(auto n: res){
                    std::cout << *n << "\n";
                }
                std::cout << m << "\n";
            }
        }
        else{
            std::cout << desc << std::endl;
        }
        
    }
    catch (const po::error &ex){
        std::cerr << ex.what() << std::endl;
    }
    return 0;
}

#ifndef _PARSER_
#define _PARSER_

#include <iostream>
#include <string>
#include <map>
#include <stdlib.h>
#include <ros/ros.h>

using namespace std;

class parser {

    map<string, string> args_map;

public:
    parser(int argc, char *argv[]) {

        try {
            if (argc == 1)
                throw 2;
            else if ((argc + 1) % 2 != 0)
                throw 1;

        }
        catch (int e) {
            if (e == 1) {
                cout << "Error: all parameters need to be in the format: '--flag_name parameter_value'" << endl;
                cout << "Example: --ply /home/user/pointcloud.ply --trim 15.0 --holemaxsize 4300.0" << endl;
            } else if (e == 2) {

                cout << "The program requires a CSV file or a PLY file to process, please use the following:" << endl;
                cout << "Example 1: --ply /home/user/mypointcloud.ply" << endl;
                cout << "Example 2: --csv /home/user/mypointcloud.csv" << endl;
            }

            exit(EXIT_FAILURE);
        }

        args_map[string("--ply")] = string(".");
        args_map[string("--csv")] = string(".");
        args_map[string("--output")] = string(".");
        args_map[string("--holemaxsize")] = string("3000.0");
        args_map[string("--trim")] = string("15.0");
        args_map[string("--sample")] = string("1.0");
        args_map[string("--gridm")] = string("1.2");
        args_map[string("--texturesize")] = string("4096");


        for (size_t i = 0; i < argc - 1; i++) {

            if (args_map.find(string(argv[i])) != args_map.end())
                args_map[string(argv[i])] = argv[i + 1];
            else if (i % 2 != 0)
                cout << "Warning: argument " << argv[i] << " not found!" << endl;
        }

        cout << "Using parameters:" << endl;
        for (std::map<string, string>::iterator it = args_map.begin(); it != args_map.end(); ++it)
            cout << it->first << " " << it->second << endl;

    }

    map<string, string> get_map() { return args_map; }
};


#endif
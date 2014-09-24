//
#ifndef _FATTREE_MESH_HPP_
#define _FATTREE_MESH_HPP_

#include "network.hpp"

class Fattree_mesh : public Network {
    int _k,_n;
    int mesh_cnt;
    int mesh_outchannel_cnt;
    void _ComputeSize(const Configuration &config);
    void _BuildNet(const Configuration &config);


public:
    Fattree_mesh(const Configuration &config, const string &name);
    static void RegisterRoutingFunctions();
    int getN() const;
    int getK() const;
}

#endif

//
#include "booksim.hpp"
#include "fattree_mesh.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include <vector>
#include <sstream>



Fattree_mesh::Fattree_mesh( const Configuration &config, const string & name) :
Network( config, name )
{
    _ComputeSize(config);
    _Alloc();
    _BuildNet(config);
}

void Fattree_mesh::RegisterRoutingFunctions(){
}

void Fattree_mesh::_ComputeSize(const Configuration &config){

}

void Fattree_mesh::_BuildNet(const Configuration &config){

}

int Fattree_mesh::getN() const
{
    return _n;
}

int Fatree_mesh::getK() const
{
    return _k;
}

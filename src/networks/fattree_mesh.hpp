//
#ifndef _FATTREE_MESH_HPP_
#define _FATTREE_MESH_HPP_

#define _FATTREE_MESH_DEBUG_

#include "network.hpp"
#include "routefunc.hpp"
#include <map>

#define MESH_K 8
#define MESH_N 2
#define FATTREE_K 2
#define FATTREE_N 6
#define MESH_OUTCHANNEL_CNT 1

class Fattree_mesh : public Network {
    int mesh_k,mesh_n,fattree_k,fattree_n;
    int mesh_cnt;	//how many meshes
    int mesh_outchannel_cnt;	//how many channels between a mesh and the fattree
	int mesh_nodes;	//how many nodes per mesh
	int fattree_switch_layer_width;//how many nodes per switch layer
	int mesh_channels;	//how many channels per mesh
	int fattree_channels;	//how many channels the big fattree has
	map<int, int> bridge_nodes;	//which node will be connected to fattree,
								//using which out channel of the mesh
	
    void _ComputeSize(const Configuration &config);
    void _BuildNet(const Configuration &config);
	
	void generateBridgeNodeSet(void);	//calculate bridge_nodes
	
	Router*& getFattreeNode(int layer, int pos);
	Router*& getMeshNode(int mesh_id, int node_id);
	int getFattreeNodeID(int layer, int pos);
	int getMeshNodeID(int mesh_id, int node_id);
	
	int getFattreeUpChannelID(int layer, int node, int pos);
	int getFattreeDownChannelID(int layer, int node, int pos);
	int getFattreeNextLayerConnectedNodeOffset(int layer, int node, int port);
	int getFattreeNextLayerConnectedNodePort(int layer, int node, int port);
	
	int getMeshLeftChannelID(int mesh_id, int node_id, int dim);
	int getMeshRightChannelID(int mesh_id, int node_id, int dim);
	int getMeshRelativeLeftNodeID(int node_id, int dim);
	int getMeshRelativeRightNodeID(int node_id, int dim);
	int getMeshOutChannelID(int mesh_id, int out_channel);
	int getMeshInChannelID(int mesh_id, int out_channel);


public:
    Fattree_mesh(const Configuration &config, const string &name);
    static void RegisterRoutingFunctions();
    int getMeshN() const;
    int getMeshK() const;
	int getFattreeN() const;
	int getFattreeK() const;
};

//routing functions

void dor_nca_fattree_mesh( const Router *r, const Flit *f, int in_channel, 
	OutputSet *outputs, bool inject ) ;

#endif

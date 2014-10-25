//
#ifndef _FATTREE_MESH_HPP_
#define _FATTREE_MESH_HPP_

#define _FATTREE_MESH_DEBUG_

#include "network.hpp"
#include "routefunc.hpp"
#include <map>

#define MESH_K 2
#define MESH_N 2
#define FATTREE_K 2
#define FATTREE_N 2
#define MESH_OUTCHANNEL_CNT 1


class Fattree_mesh : public Network {


public:
    int mesh_k,mesh_n,fattree_k,fattree_n;
    int mesh_cnt;	//how many meshes
    int mesh_outchannel_cnt;	//how many channels between a mesh and the fattree
	int mesh_nodes;	//how many nodes per mesh
	int fattree_switches;	//how many switches the big fattree has
	int fattree_switch_layer_width;//how many nodes per switch layer
	int mesh_channels;	//how many channels per mesh
	int fattree_channels;	//how many channels the big fattree has
	
	map<int, int> bridge_nodes; //which node will be connected to fattree,
	//using which out channel of the mesh. index: node_id, channel_id
	vector<pair<int, int> > bridge_nodes_list;	//vector's index is meaningless.
	//stores the map in a sequential way, so we can access the element randomly.

	map<int, int> chan_src_ix;	//a channel's index in its source's channel array
	map<int, int> chan_sink_ix;	//a channel's index in its sink's channel array
	map<int, int> term_chan_src_ix;	//a terminal channel's index in its source's channel array
	map<int, int> term_chan_sink_ix;//a terminal channel's index in its sink's channel array

private:
    void _ComputeSize(const Configuration &config);
    void _BuildNet(const Configuration &config);
	
	void generateBridgeNodeSet(void);	//calculate bridge_nodes
	
	Router*& getFattreeNode(int layer, int pos);
	Router*& getMeshNode(int mesh_id, int node_id);

public:
	int getFattreeNodeID(int layer, int pos) const;

	int getFattreeUpChannelID(int layer, int node, int pos) const;
	int getFattreeDownChannelID(int layer, int node, int pos) const;

	int getFattreeNextLayerConnectedNodeOffset(int layer, int node, int port) const;
	int getFattreeNextLayerConnectedNodePort(int layer, int node, int port) const;
	

	int getMeshNodeID(int mesh_id, int node_id) const;

	int getMeshLeftChannelID(int mesh_id, int node_id, int dim) const;
	int getMeshRightChannelID(int mesh_id, int node_id, int dim) const;

	int getMeshRelativeLeftNodeID(int node_id, int dim) const;
	int getMeshRelativeRightNodeID(int node_id, int dim) const;

	int getMeshOutChannelID(int mesh_id, int out_channel) const;
	int getMeshInChannelID(int mesh_id, int out_channel) const;

	int getMeshTerminalChannelID(int mesh_id, int node_id) const;


//public:
    Fattree_mesh(const Configuration &config, const string &name);
    static void RegisterRoutingFunctions();

    int getMeshN() const;
    int getMeshK() const;
	int getFattreeN() const;
	int getFattreeK() const;
	void printTopo() const;		//for debug use
};

//routing functions

void dor_nca_fattree_mesh( const Router *r, const Flit *f, int in_channel, 
	OutputSet *outputs, bool inject , const Network *net1=NULL) ;

#endif

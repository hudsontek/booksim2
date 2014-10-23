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

//for simplicity, the size values are obtained from the macro define
void Fattree_mesh::_ComputeSize(const Configuration &config){
    mesh_k = MESH_K;
    mesh_n = MESH_N;
    fattree_k = FATTREE_K;
    fattree_n = FATTREE_N;
    mesh_cnt = powi(fattree_k, fattree_n);
    mesh_outchannel_cnt = MESH_OUTCHANNEL_CNT;

    //gK, gN are global variables used by routing functions
    gK = fattree_k;//not sure
    gN = fattree_n;//not sure

    //_nodes,_size,_channels are variables defined in network.hpp
    //these variables need CAREFUL EXAMINATION!!!
    mesh_nodes = powi(mesh_k,mesh_n);
    mesh_channels = mesh_nodes * mesh_n * 2;    //every node has 2 out channels within every dimension
    fattree_switch_layer_width = powi(fattree_k, fattree_n - 1);
    fattree_switches = fattree_n * fattree_switch_layer_width;
    fattree_channels = (2 * fattree_k * fattree_switch_layer_width)
		*(fattree_n - 1);	//since first level has only downside channels, 
    //and last level has only upside channels, 
    //the combined effect is equal to (fattree_n -1) intact switch layers

    _nodes = mesh_nodes * mesh_cnt;   //# of injecting/ejecting nodes. this variable will also be used to establish inject/eject channels
    _size = _nodes + fattree_switches;    //# of routers/switches
    _channels = mesh_channels * mesh_cnt 
		+ fattree_channels 
		+ 2 * mesh_outchannel_cnt * mesh_cnt;  //the UNIDIMENSIONAL channel count
    //channels are numbered in this order: first is fattree's, 
    //then meshes's, then channels from mesh to fattree, 
    //at last is channels from fattree to mesh

//    gNodes = _nodes;
//    gSize = _size;
//    gChannels = _channels;

#ifdef _FATTREE_MESH_DEBUG_
    cout << "_nodes=" << _nodes << endl;
    cout << "_size=" << _size << endl;
    cout << "_channels=" << _channels << endl;
#endif
}

void Fattree_mesh::_BuildNet(const Configuration &config){
    int chan_ix;    //index into the _channel array
    int node_ix;    //id of the switch/router node
    int degree;	    //how many bidirectional channels the router has
    int node_offset;//offset of next layer's node
    int node_port;  //port number of next layer's node
    ostringstream name;

    //instantiate nodes in fattree
    for(int layer = 0;layer < fattree_n; ++layer)
    {
		for(int node = 0; node < fattree_switch_layer_width; ++node)
		{
		    if(layer == 0)  //first layer has only downward channels
				degree = fattree_k;
		    else if(layer == fattree_n - 1) //last layer's downward channels are bridge channels
				degree = fattree_k + fattree_k * mesh_outchannel_cnt;
		    else
				degree = fattree_k * 2;

		    node_ix = getFattreeNodeID(layer, node);
		    
		    name.str("");//clear out the string
		    name << "router_in_fattree:loc_" << layer << "_" << node;

		    Router *r = Router::NewRouter( config, this, name.str( ), 
				    node_ix,degree, degree ,this );
		    getFattreeNode(layer, node) = r;
		    _timed_modules.push_back(r);

		}
    }//end of instantiate fattree nodes

    // within the fattree, connect switch nodes to channels
    for(int layer = 0; layer < fattree_n; ++layer)
    {
		for(int node = 0; node < fattree_switch_layer_width; ++node)
		{
		    for(int port = 0; port < fattree_k; ++port)
		    {
				if(layer > 0)//this router isn't @ top level, connect upside channels
				{
				    //attach out channel to this node
				    chan_ix = getFattreeUpChannelID(layer, node, port);
				    getFattreeNode(layer, node)->AddOutputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);
				    _chan[chan_ix]->SetLatency(1);
				    _chan_cred[chan_ix]->SetLatency(1);
				}

				if(layer < fattree_n - 1)//this router isn't @ bottom level, connect downside channels
				{
				    //attach out channel to this node
				    chan_ix = getFattreeDownChannelID(layer, node, port);
				    getFattreeNode(layer, node)->AddOutputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);
				    _chan[chan_ix]->SetLatency(1);
				    _chan_cred[chan_ix]->SetLatency(1);

				    //connect this out channel to the related node in next layer
				    node_offset = getFattreeNextLayerConnectedNodeOffset(layer, node, port);
				    node_port =	  getFattreeNextLayerConnectedNodePort(layer, node, port);
				    getFattreeNode(layer + 1, node_offset)->AddInputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);

				    //connect out channel of the related node in next layer to this node
				    chan_ix = getFattreeUpChannelID(layer + 1, node_offset, node_port);
				    getFattreeNode(layer, node)->AddInputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);
				}
		    }
		}
    }//end of connect switch nodes to channels in fattree

    assert(mesh_k > 1);
    map<int,int>::const_iterator iter; //iterator into the bridgeNode set
    int linput, rinput, loutput, routput, lnode, rnode;

    //instantiate nodes in every mesh, and attach channels to them
    generateBridgeNodeSet();

    for(int mesh = 0; mesh < mesh_cnt; ++mesh)
    {
		for(int node = 0; node < mesh_nodes; ++node)
		{
		    name.str("");	//reset name string
		    name << "router_in_mesh_" << mesh << "_loc";
		    for(int dim_offset = mesh_nodes / mesh_k; 
				    dim_offset >= 1; dim_offset /= mesh_k)  //generate node name
		    {
				name << "_" << (node / dim_offset) % mesh_k;
		    }

		    degree = 2 * mesh_n + 1;	//"1" is for the inject/eject channel
		    iter = bridge_nodes.find(node);
		    if(iter != bridge_nodes.end())  //this node will connect to fattree
//				degree += bridge_nodes.size();	//a bug is fixed here
		    	degree += 1;	//"1" is for the channel connected to fattree

		    Router *r = Router::NewRouter(config, this, name.str(), 
				    getMeshNodeID(mesh, node), degree, degree, this);
		    getMeshNode(mesh, node) = r;
		    _timed_modules.push_back(r);
		    
		    //connect inject and eject channels
		    chan_ix = mesh * mesh_nodes + node;	//index of inject & eject channels
		    r->AddInputChannel(_inject[chan_ix],_inject_cred[chan_ix]);
		    r->AddOutputChannel(_eject[chan_ix],_eject_cred[chan_ix]);
		    _inject[chan_ix]->SetLatency(1);
		    _inject_cred[chan_ix]->SetLatency(1);//not sure
		    _eject[chan_ix]->SetLatency(1);
		    _eject_cred[chan_ix]->SetLatency(1);//not sure

		    //within the mesh, attach channels to node
		    for(int dim = 0; dim < mesh_n; ++dim)
		    {
				lnode = getMeshRelativeLeftNodeID(node, dim);
				rnode = getMeshRelativeRightNodeID(node, dim);
				loutput = getMeshLeftChannelID(mesh, node, dim);
				routput = getMeshRightChannelID(mesh, node, dim);
				linput = getMeshRightChannelID(mesh, lnode, dim);
				rinput = getMeshLeftChannelID(mesh, rnode, dim);

				r->AddOutputChannel(_chan[loutput], _chan_cred[loutput]);
				r->AddOutputChannel(_chan[routput], _chan_cred[routput]);
				r->AddInputChannel(_chan[linput],_chan_cred[linput]);
				r->AddInputChannel(_chan[rinput],_chan_cred[rinput]);

				//set channel latency, no need for input channels because
				//this node's input is other node's output
				_chan[loutput]->SetLatency(1);
				_chan[routput]->SetLatency(1);
				_chan_cred[loutput]->SetLatency(1);
				_chan_cred[routput]->SetLatency(1);

//				_chan[linput]->SetLatency(1);
//				_chan[rinput]->SetLatency(1);
//				_chan_cred[linput]->SetLatency(1);
//				_chan_cred[rinput]->SetLatency(1);
		    }
		}//end of iteration in a mesh
    }//end of instantiate nodes in meshes

#ifdef _FATTREE_MESH_DEBUG_
    //print out the topology
    cout << "before the bridge channels are connected." << endl;

    for(int i = 0; i < _size; ++i)
    {
		cout << _routers[i]->Name() << " has " << _routers[i]->NumInputs() << " input channels:" << endl;
		for(int j = 0; j < _routers[i]->NumInputs(); ++j)
		{
				if(!_routers[i]->GetInputChannel(j))
				{
						cout << "\tnot available yet." << endl;
						continue;
				}
		    cout << "\t" << _routers[i]->GetInputChannel(j)->Name() << endl;
		}
		cout << endl;

		cout << _routers[i]->Name() << " has " << _routers[i]->NumOutputs() << " output channels:" << endl;
		for(int j = 0; j < _routers[i]->NumOutputs(); ++j)
		{
				if(!_routers[i]->GetInputChannel(j))
				{
						cout << "\tnot available yet." << endl;
						continue;
				}
		    cout << "\t" << _routers[i]->GetOutputChannel(j)->Name() << endl;
		}

		cout << "===============" << endl;
    }

    cout << "======================================================" << endl;
#endif

    //so far, we have neither attached bridge channels to fattree nor meshes

    //connect bridge channels to nodes in fattree's last layer
    int m2f_chan, f2m_chan;
    int mesh;
    for(int node = 0; node < fattree_switch_layer_width; ++node)//which node in last layer
    {
		for(int node_port = 0; node_port < fattree_k; ++node_port)
		{
		    mesh = node * fattree_k + node_port;
		    for(iter = bridge_nodes.begin(); iter != bridge_nodes.end(); ++iter)
		    {
				m2f_chan = getMeshOutChannelID(mesh, iter->second);
				f2m_chan = getMeshInChannelID(mesh, iter->second);

				getFattreeNode(fattree_n - 1, node)->AddOutputChannel(_chan[f2m_chan],_chan_cred[f2m_chan]);
				getFattreeNode(fattree_n - 1, node)->AddInputChannel(_chan[m2f_chan],_chan_cred[m2f_chan]);

				getMeshNode(mesh, iter->first)->AddOutputChannel(_chan[m2f_chan],_chan_cred[m2f_chan]);
				getMeshNode(mesh, iter->first)->AddInputChannel(_chan[f2m_chan],_chan_cred[f2m_chan]);

				_chan[m2f_chan]->SetLatency(1);
				_chan[f2m_chan]->SetLatency(1);
				_chan_cred[m2f_chan]->SetLatency(1);
				_chan_cred[f2m_chan]->SetLatency(1);
		    }
		}
    }//end of connect bridge channels between fattree and meshes

#ifdef _FATTREE_MESH_DEBUG_
    //print out the topology
    cout << "after the whole topology is completed." << endl;

    for(int i = 0; i < _size; ++i)
    {
		cout << _routers[i]->Name() << " has " << _routers[i]->NumInputs() << " input channels:" << endl;
		for(int j = 0; j < _routers[i]->NumInputs(); ++j)
		{
		    cout << "\t" << _routers[i]->GetInputChannel(j)->Name() << endl;
		}
		cout << endl;

		cout << _routers[i]->Name() << " has " << _routers[i]->NumOutputs() << " output channels:" << endl;
		for(int j = 0; j < _routers[i]->NumOutputs(); ++j)
		{
		    cout << "\t" << _routers[i]->GetOutputChannel(j)->Name() << endl;
		}

		cout << "===============" << endl;
    }

    cout << "======================================================" << endl;

#endif

}//end of _BuildNet()

//currently this function is rather simple
void Fattree_mesh::generateBridgeNodeSet(void){
    assert(mesh_outchannel_cnt <= mesh_nodes);
    for(int ix = 0; ix < mesh_outchannel_cnt; ++ix)
    {
		bridge_nodes[ix] = ix;
    }
}

Router*& Fattree_mesh::getFattreeNode(int layer, int pos){
    return _routers[layer*fattree_switch_layer_width 
		+ pos];	//we number the fattree node first
}

Router*& Fattree_mesh::getMeshNode(int mesh_id, int node_id){
    return _routers[fattree_n * fattree_switch_layer_width + 
		mesh_id * mesh_nodes + node_id];
}

int Fattree_mesh::getFattreeNodeID(int layer, int pos){
    return layer * fattree_switch_layer_width + pos;
}

int Fattree_mesh::getMeshNodeID(int mesh_id, int node_id){
    return fattree_n * fattree_switch_layer_width 
		+ mesh_id * mesh_nodes + node_id;
}

int Fattree_mesh::getFattreeUpChannelID(int layer, int node, int pos){
    assert(layer > 0 
		    && node < fattree_switch_layer_width 
		    && pos < fattree_k);
    int per_layer = fattree_k * fattree_switch_layer_width;
    return pos + node * fattree_k 
		+ per_layer + (layer - 1) * per_layer * 2;
}

int Fattree_mesh::getFattreeDownChannelID(int layer, int node, int pos){
    assert(layer < fattree_n - 1 
		    && node < fattree_switch_layer_width 
		    && pos < fattree_k);
    int per_layer = fattree_k * fattree_switch_layer_width;
    //the DownChannelID is ahead of UpChannelID by per_layer
    return pos + node * fattree_k + layer * per_layer * 2;
}


int Fattree_mesh::getFattreeNextLayerConnectedNodeOffset(int layer, int node, int port){
    assert(layer < fattree_n - 1 
		    && node < fattree_switch_layer_width 
		    && port < fattree_k);
    int size = powi(fattree_k, fattree_n - 2 - layer);	//how many nodes per cluster
    int cluster = (node / size) % fattree_k;	//which cluster this node belongs to
    int offset = node - cluster * size + port * size;	//which node of the layer
    return offset;
}

int Fattree_mesh::getFattreeNextLayerConnectedNodePort(int layer, int node, int port){
    assert(layer < fattree_n - 1 
		    && node < fattree_switch_layer_width 
		    && port < fattree_k);
    int size = powi(fattree_k, fattree_n - 2 - layer);
    int cluster = (node / size) % fattree_k;
    return cluster;
}

//the channels are numbered in this way: node 0's right channel in dimension 0 is 0, 
//left is 1, right channel in dimension 1 is 2...
//node 1's right channel in dimension 0 is 2*_n, left is 2*_n+1...
//channels are numbered node by node, and dimension by dimension of the same node, 
//and left is prior to right
int Fattree_mesh::getMeshLeftChannelID(int mesh_id, int node_id, int dim){
    int base = fattree_channels 
		+ mesh_id * mesh_channels 
		+ 2 * mesh_n * node_id;
    int offset = 2 * dim;   //left is prior to right
    return base + offset;
}

int Fattree_mesh::getMeshRightChannelID(int mesh_id, int node_id, int dim){
    int base = fattree_channels 
		+ mesh_id * mesh_channels 
		+ 2 * mesh_n * node_id;
    int offset = 2 * dim + 1;
    return base + offset;
}

int Fattree_mesh::getMeshRelativeLeftNodeID( int node_id, int dim){
    int k_exp_dim = powi(mesh_k, dim);
    int loc_in_dim = (node_id / k_exp_dim ) % mesh_k;
    if(loc_in_dim == 0)	//wrap around to the rightmost node within this dim
		return node_id + (mesh_k - 1)*k_exp_dim;
    return node_id - k_exp_dim;	//left is prior to right
}

int Fattree_mesh::getMeshRelativeRightNodeID( int node_id, int dim){
    int k_exp_dim = powi(mesh_k, dim);
    int loc_in_dim = (node_id / k_exp_dim ) % mesh_k;
    if(loc_in_dim == mesh_k - 1)//wrap around to the leftmost node within this dim
		return node_id - (mesh_k - 1)*k_exp_dim;
    return node_id + k_exp_dim;	//left is prior to right
}

int Fattree_mesh::getMeshOutChannelID(int mesh_id, int out_channel){
    int base = fattree_channels + mesh_channels * mesh_cnt;
    int offset = mesh_id * mesh_outchannel_cnt + out_channel;
    return base + offset;
}

int Fattree_mesh::getMeshInChannelID(int mesh_id, int out_channel){
    int base = fattree_channels + mesh_channels * mesh_cnt 
		+ mesh_outchannel_cnt * mesh_cnt;
    int offset = mesh_id * mesh_outchannel_cnt + out_channel;
    return base + offset;
}

void Fattree_mesh::RegisterRoutingFunctions(){
    gRoutingFunctionMap["dor_nca_fattree_mesh"] = &dor_nca_fattree_mesh;
}

int Fattree_mesh::getMeshK() const
{
    return mesh_k;
}

int Fattree_mesh::getMeshN() const
{
    return mesh_n;
}

int Fattree_mesh::getFattreeK() const
{
    return fattree_k;
}

int Fattree_mesh::getFattreeN() const
{
    return fattree_n;
}

//a dummy routing function
void dor_nca_fattree_mesh( const Router *r, const Flit *f, int in_channel, 
		OutputSet *outputs, bool inject, const Network *net1)
{
	//common part of a routing function
	int vcBegin = 0, vcEnd = gNumVCs - 1;
	if (f->type == Flit::READ_REQUEST) {
		vcBegin = gReadReqBeginVC;
		vcEnd = gReadReqEndVC;
	} else if (f->type == Flit::WRITE_REQUEST) {
		vcBegin = gWriteReqBeginVC;
		vcEnd = gWriteReqEndVC;
	} else if (f->type == Flit::READ_REPLY) {
		vcBegin = gReadReplyBeginVC;
		vcEnd = gReadReplyEndVC;
	} else if (f->type == Flit::WRITE_REPLY) {
		vcBegin = gWriteReplyBeginVC;
		vcEnd = gWriteReplyEndVC;
	}
	assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

	Fattree_mesh *net = (Fattree_mesh *)net1;	//the explicit type cast is necessary

	int out_port;

	if (inject)
	{
		out_port = -1;
	}
	else	//out_port needs to be calculated
	{
		int dest = f->dest;
		int loc = r->GetID();

		assert(loc < gSize);
		int fattree_width = powi(FATTREE_K, FATTREE_N - 1);
		int fattree_switches = FATTREE_N * fattree_width;
		int mesh_nodes = powi(MESH_K, MESH_N);

		int loc_mesh = (loc - fattree_switches) / mesh_nodes;
		int dest_mesh = (dest - fattree_switches) / mesh_nodes;

		if(loc < fattree_switches)	//this is a fattree node
		{
			int level = loc / fattree_width;
			int pos = loc % fattree_width;
			int routers_per_cluster = powi(FATTREE_K, FATTREE_N - level - 1);
			int cluster = pos / routers_per_cluster;
			int nodes_per_cluster = powi(FATTREE_K, FATTREE_N - level);

			if(dest_mesh < (cluster + 1) * nodes_per_cluster	//we've reached NCA, going down
					&& dest_mesh >= cluster * nodes_per_cluster)
			{
				if(level == FATTREE_N - 1)	//lowest level
				{
					out_port = dest_mesh % FATTREE_K;
				}
				else
				{
					int nodes_per_branch = nodes_per_cluster / FATTREE_K;
					int offset_in_cluster = dest_mesh % nodes_per_cluster;
					out_port = offset_in_cluster / nodes_per_branch;
				}
			}
			else	//destination is in other subtree, going up
			{
//				assert(in_channel < FATTREE_K);

			}
		}
		else	//this is a mesh node
		{
			if(loc_mesh == dest_mesh)	//reach the destination mesh
			{
				if(loc == dest)	//reach the destination node
				{
					out_port = r->NumOutputs();	//not sure
				}
				else	//use dor within this mesh to reach the destination node
				{
					loc %= mesh_nodes;
					dest %= mesh_nodes;
					int dim;
					for(dim = 0; dim < MESH_N - 1; ++dim)
					{
						if(loc % MESH_K != dest % MESH_K)	//loc and dest are not @ the same location within this dimension
							break;
						loc /= MESH_K;
						dest /= MESH_K;
					}

					loc %= MESH_K;
					dest %= MESH_K;

					if(loc < dest)
						out_port = 2 * dim + 1;	//move right
					else
						out_port = 2 * dim;		//move left
				}
			}
			else	//we should get out of this mesh first
			{

			}
		}
	}

	outputs->Clear();
	outputs->AddRange(out_port, vcBegin, vcEnd);
}

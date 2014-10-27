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
    mesh_k = config.GetInt("mesh_k");
    mesh_n = config.GetInt("mesh_n");
    fattree_k = config.GetInt("fattree_k");
    fattree_n = config.GetInt("fattree_n");
    mesh_cnt = powi(fattree_k, fattree_n);
    mesh_outchannel_cnt = config.GetInt("mesh_out_channels");

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

    gNodes = _nodes;
    gSize = _size;
    gChannels = _channels;

    chan_src_ix.assign(_channels, 0);
    chan_sink_ix.assign(_channels, 0);
    term_chan_src_ix.assign(_nodes, 0);
    term_chan_sink_ix.assign(_nodes, 0);

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

    // within the fattree, connect channels to switch nodes
    for(int layer = 0; layer < fattree_n; ++layer)
    {
		for(int node = 0; node < fattree_switch_layer_width; ++node)
		{
		    for(int port = 0; port < fattree_k; ++port)
		    {
				if(layer < fattree_n - 1)//this router isn't @ bottom level, connect downside channels
				{
				    //attach out channel to this node
				    chan_ix = getFattreeDownChannelID(layer, node, port);
				    getFattreeNode(layer, node)->AddOutputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);
				    chan_src_ix[chan_ix] =
				    		getFattreeNode(layer, node)->NumConnectedOutputs() - 1;
				    _chan[chan_ix]->SetLatency(1);
				    _chan_cred[chan_ix]->SetLatency(1);

				    //connect this out channel to the related node in next layer
				    node_offset = getFattreeNextLayerConnectedNodeOffset(layer, node, port);
				    node_port =	  getFattreeNextLayerConnectedNodePort(layer, node, port);

				    getFattreeNode(layer + 1, node_offset)->AddInputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);
				    chan_sink_ix[chan_ix] =
				    		getFattreeNode(layer + 1, node_offset)->NumConnectedInputs() - 1;

				    //connect out channel of the related node in next layer to this node
				    chan_ix = getFattreeUpChannelID(layer + 1, node_offset, node_port);
				    getFattreeNode(layer, node)->AddInputChannel(_chan[chan_ix],
						    _chan_cred[chan_ix]);
				    chan_sink_ix[chan_ix] =
				    		getFattreeNode(layer, node)->NumConnectedInputs() - 1;
				}

				if(layer > 0)//this router isn't @ top level, connect upside channels
				{
					//attach out channel to this node
					chan_ix = getFattreeUpChannelID(layer, node, port);
					getFattreeNode(layer, node)->AddOutputChannel(_chan[chan_ix],
							_chan_cred[chan_ix]);
					chan_src_ix[chan_ix] = getFattreeNode(layer, node)->NumConnectedOutputs() - 1;
					_chan[chan_ix]->SetLatency(1);
					_chan_cred[chan_ix]->SetLatency(1);
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
		    	degree += 1;	//"1" is for the channel connected to fattree

		    Router *r = Router::NewRouter(config, this, name.str(), 
				    getMeshNodeID(mesh, node), degree, degree, this);
		    getMeshNode(mesh, node) = r;
		    _timed_modules.push_back(r);
		    
		    //connect inject and eject channels
		    chan_ix = mesh * mesh_nodes + node;	//index of inject & eject channels

		    r->AddInputChannel(_inject[chan_ix],_inject_cred[chan_ix]);
		    term_chan_sink_ix[chan_ix] = r->NumConnectedInputs() - 1;

		    r->AddOutputChannel(_eject[chan_ix],_eject_cred[chan_ix]);
		    term_chan_src_ix[chan_ix] = r->NumConnectedOutputs() - 1;

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
				chan_src_ix[loutput] = r->NumConnectedOutputs() - 1;

				r->AddOutputChannel(_chan[routput], _chan_cred[routput]);
				chan_src_ix[routput] = r->NumConnectedOutputs() - 1;

				r->AddInputChannel(_chan[linput],_chan_cred[linput]);
				chan_sink_ix[linput] = r->NumConnectedInputs() - 1;

				r->AddInputChannel(_chan[rinput],_chan_cred[rinput]);
				chan_sink_ix[rinput] = r->NumConnectedInputs() - 1;


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

//#ifdef _FATTREE_MESH_DEBUG_
//    //print out the topology
//    cout << "before the bridge channels are connected." << endl;
//
//    printTopo();
//
//#endif

    //so far, we have neither attached bridge channels to fattree nor meshes

    //connect bridge channels to nodes in fattree's last layer
    int m2f_chan, f2m_chan;
    int mesh;
    Router *r;
    for(int node = 0; node < fattree_switch_layer_width; ++node)//which node in last layer
    {
		for(int node_port = 0; node_port < fattree_k; ++node_port)
		{
		    mesh = node * fattree_k + node_port;
		    for(iter = bridge_nodes.begin(); iter != bridge_nodes.end(); ++iter)
		    {
				m2f_chan = getMeshOutChannelID(mesh, iter->second);
				f2m_chan = getMeshInChannelID(mesh, iter->second);

				r = getFattreeNode(fattree_n - 1, node);
				r->AddOutputChannel(_chan[f2m_chan],_chan_cred[f2m_chan]);
				chan_src_ix[f2m_chan] = r->NumConnectedOutputs() - 1;
				r->AddInputChannel(_chan[m2f_chan],_chan_cred[m2f_chan]);
				chan_sink_ix[m2f_chan] = r->NumConnectedInputs() - 1;

				r = getMeshNode(mesh, iter->first);
				r->AddOutputChannel(_chan[m2f_chan],_chan_cred[m2f_chan]);
				chan_src_ix[m2f_chan] = r->NumConnectedOutputs() - 1;
				r->AddInputChannel(_chan[f2m_chan],_chan_cred[f2m_chan]);
				chan_sink_ix[f2m_chan] = r->NumConnectedInputs() - 1;

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

    printTopo();

#endif

}//end of _BuildNet()

//currently this function is rather simple
void Fattree_mesh::generateBridgeNodeSet(void){
    assert(mesh_outchannel_cnt <= mesh_nodes);
    for(int ix = 0; ix < mesh_outchannel_cnt; ++ix)
    {
		bridge_nodes[ix] = ix;
		bridge_nodes_list.push_back(make_pair(ix, ix));
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

int Fattree_mesh::getFattreeNodeID(int layer, int pos) const{
    return layer * fattree_switch_layer_width + pos;
}

int Fattree_mesh::getMeshNodeID(int mesh_id, int node_id) const{
    return fattree_n * fattree_switch_layer_width 
		+ mesh_id * mesh_nodes + node_id;
}

int Fattree_mesh::getFattreeUpChannelID(int layer, int node, int pos) const{
    assert(layer > 0 
    		&& layer < fattree_n
		    && node < fattree_switch_layer_width 
		    && pos < fattree_k);
    int per_layer = fattree_k * fattree_switch_layer_width;
    return pos + node * fattree_k 
		+ per_layer + (layer - 1) * per_layer * 2;
}

int Fattree_mesh::getFattreeDownChannelID(int layer, int node, int pos) const{
    assert(layer >= 0
    		&& layer < fattree_n - 1
		    && node < fattree_switch_layer_width 
		    && pos < fattree_k);
    int per_layer = fattree_k * fattree_switch_layer_width;
    //the DownChannelID is ahead of UpChannelID by per_layer
    return pos + node * fattree_k + layer * per_layer * 2;
}


int Fattree_mesh::getFattreeNextLayerConnectedNodeOffset(int layer, int node, int port) const{
    assert(layer < fattree_n - 1 
		    && node < fattree_switch_layer_width 
		    && port < fattree_k);
    int size = powi(fattree_k, fattree_n - 2 - layer);	//how many nodes per cluster
    int cluster = (node / size) % fattree_k;	//which cluster this node belongs to
    int offset = node - cluster * size + port * size;	//which node of the layer
    return offset;
}

int Fattree_mesh::getFattreeNextLayerConnectedNodePort(int layer, int node, int port) const{
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
int Fattree_mesh::getMeshLeftChannelID(int mesh_id, int node_id, int dim) const{
    int base = fattree_channels 
		+ mesh_id * mesh_channels 
		+ 2 * mesh_n * node_id;
    int offset = 2 * dim;   //left is prior to right
    return base + offset;
}

int Fattree_mesh::getMeshRightChannelID(int mesh_id, int node_id, int dim) const{
    int base = fattree_channels 
		+ mesh_id * mesh_channels 
		+ 2 * mesh_n * node_id;
    int offset = 2 * dim + 1;
    return base + offset;
}

int Fattree_mesh::getMeshRelativeLeftNodeID( int node_id, int dim) const{
    int k_exp_dim = powi(mesh_k, dim);
    int loc_in_dim = (node_id / k_exp_dim ) % mesh_k;
    if(loc_in_dim == 0)	//wrap around to the rightmost node within this dim
		return node_id + (mesh_k - 1)*k_exp_dim;
    return node_id - k_exp_dim;	//left is prior to right
}

int Fattree_mesh::getMeshRelativeRightNodeID( int node_id, int dim) const{
    int k_exp_dim = powi(mesh_k, dim);
    int loc_in_dim = (node_id / k_exp_dim ) % mesh_k;
    if(loc_in_dim == mesh_k - 1)//wrap around to the leftmost node within this dim
		return node_id - (mesh_k - 1)*k_exp_dim;
    return node_id + k_exp_dim;	//left is prior to right
}

int Fattree_mesh::getMeshOutChannelID(int mesh_id, int out_channel) const{
    int base = fattree_channels + mesh_channels * mesh_cnt;
    int offset = mesh_id * mesh_outchannel_cnt + out_channel;
    return base + offset;
}

int Fattree_mesh::getMeshInChannelID(int mesh_id, int out_channel) const{
    int base = fattree_channels + mesh_channels * mesh_cnt 
		+ mesh_outchannel_cnt * mesh_cnt;
    int offset = mesh_id * mesh_outchannel_cnt + out_channel;
    return base + offset;
}

int Fattree_mesh::getMeshTerminalChannelID(int mesh_id, int node_id) const{
	return mesh_id * mesh_nodes + node_id;
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

void Fattree_mesh::printTopo() const
{
	for(int i = 0; i < _size; ++i)
	{
		const FlitChannel *fc;
		const Router *r;

//		//print the input channels
//		cout << _routers[i]->Name() << " has " << _routers[i]->NumInputs() << " input channels:" << endl;
//		for(int j = 0; j < _routers[i]->NumInputs(); ++j)
//		{
//			fc = _routers[i]->GetInputChannel(j);
//			if(!fc)
//			{
//				cout << "\tchannel_not_avail." << endl;
//				continue;
//			}
//
//			cout << "\t" << fc->Name() << " from ";
//
//			r = fc->GetSource();
//			if(!r)
//			{
//				cout << "router_not_avail." <<endl;
//				continue;
//			}
//			cout << r->Name() << endl;
//		}
//		cout << endl;


		//print the output channels
		cout << _routers[i]->Name() << " has " << _routers[i]->NumOutputs() << " output channels:" << endl;
		for(int j = 0; j < _routers[i]->NumOutputs(); ++j)
		{
			fc = _routers[i]->GetOutputChannel(j);
			if(!fc)
			{
				cout << "\tchannel_not_avail." << endl;
				continue;
			}

			cout << "\t" << fc->Name() << " to ";

			r = fc->GetSink();
			if(!r)
			{
				cout << "router_not_avail." <<endl;
				continue;
			}
			cout << r->Name() << endl;
		}

		cout << "===============" << endl;
	}

	cout << "======================================================" << endl;
}

//routing function, the out_port is index into Router's _input_channels,
//_output_channels, _input_credits, _output_credits, so these channels
//must be added to Router in a certain order, so we can calculate it here.
//Flit's dest variable is dest's node_id, not router's id.
void dor_nca_fattree_mesh( const Router *r, const Flit *f, int in_channel, 
		OutputSet *outputs, bool inject, const Network *net1)
{
	//common part of the routing function
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

	assert(net1);
	const Fattree_mesh *net = (Fattree_mesh *)net1;	//the explicit type cast is necessary

	int out_port;

	if (inject)
	{
		out_port = -1;
	}
	else	//out_port needs to be calculated
	{
		int dest = f->dest;		//destination's node_id
		int loc = r->GetID();	//current position's router_id

		assert(loc < gSize);
		assert(dest < gNodes);

		int loc_mesh = (loc - net->fattree_switches) / net->mesh_nodes;	//may be negative
		int dest_mesh = dest / net->mesh_nodes;

		int chan_id;

		if(loc < net->fattree_switches)	//this is a fattree node
		{
			int level = loc / net->fattree_switch_layer_width;
			int pos = loc % net->fattree_switch_layer_width;
			int routers_per_cluster = powi(net->fattree_k, net->fattree_n - level - 1);
			int cluster = pos / routers_per_cluster;
			int nodes_per_cluster = powi(net->fattree_k, net->fattree_n - level);

			if(dest_mesh < (cluster + 1) * nodes_per_cluster	//we've reached NCA, going down
					&& dest_mesh >= cluster * nodes_per_cluster)
			{
				if(level == net->fattree_n - 1)	//lowest level
				{
					//the situation is a little complicate here if we want the shortest path
					//randomly choose a bridge node, could this cause dead lock?
					chan_id = net->getMeshInChannelID(dest_mesh,
							RandomInt(net->mesh_outchannel_cnt - 1));
					out_port = net->getChanSrcIx(chan_id);
				}
				else	//not lowest level
				{
					int nodes_per_branch = nodes_per_cluster / net->fattree_k;
					int offset_in_cluster = dest_mesh % nodes_per_cluster;
					int branch = offset_in_cluster / nodes_per_branch;
					chan_id = net->getFattreeDownChannelID(level, pos, branch);
					out_port = net->getChanSrcIx(chan_id);
				}
			}
			else	//destination is in other subtree, going up
			{
				chan_id = net->getFattreeUpChannelID(level,
						pos, RandomInt(net->fattree_k - 1));
				out_port = net->getChanSrcIx(chan_id);
			}
		}//end of this is a fattree node
		else	//this is a mesh node
		{
			loc = (loc - net->fattree_switches) % net->mesh_nodes;	//get the relative id within the mesh
			dest %= net->mesh_nodes;//get the relative id within the mesh

			if(loc_mesh == dest_mesh)	//reach the destination mesh
			{
				if(loc == dest)	//reach the destination node
				{
					chan_id = loc_mesh * net->mesh_nodes + loc;	//inject/eject channel id
					out_port = net->getTermChanSrcIx(chan_id);
				}
				else	//use dor within this mesh to reach the destination node
				{
					int dim;
					int loc_tmp = loc, dest_tmp = dest;	//the relative position in the dimension in which loc and dest are not @ the same location

					for(dim = 0; dim < net->mesh_n - 1; ++dim)
					{
						if(loc_tmp % net->mesh_k != dest_tmp % net->mesh_k)	//loc and dest are not @ the same location within this dimension
							break;
						loc_tmp /= net->mesh_k;
						dest_tmp /= net->mesh_k;
					}

					loc_tmp %= net->mesh_k;
					dest_tmp %= net->mesh_k;

					if(loc_tmp < dest_tmp)	//move right
					{
						chan_id = net->getMeshRightChannelID(loc_mesh, loc, dim);
					}
					else			//move left
					{
						chan_id = net->getMeshLeftChannelID(loc_mesh, loc, dim);
					}

					out_port = net->getChanSrcIx(chan_id);
				}
			}
			else	//loc_mesh != dest_mesh
			{
				map<int,int>::const_iterator iter; //iterator into the bridgeNode set

				iter = net->bridge_nodes.find(loc);
				if(iter != net->bridge_nodes.end())	//current node is a bridge node
				{
					chan_id = net->getMeshOutChannelID(loc_mesh, iter->second);
					out_port = net->getChanSrcIx(chan_id);
				}
				else	//current node isn't a bridge node, we need to reach one first
				{
					//the situation is a little complicate here if we want the shortest path
					//randomly choose a bridge node
					dest = net->bridge_nodes_list[ RandomInt(net->bridge_nodes_list.size() - 1) ].first;

					int dim;
					int loc_tmp = loc, dest_tmp = dest;	//the relative position in the dimension in which loc and dest are not @ the same location

					for(dim = 0; dim < net->mesh_n - 1; ++dim)
					{
						if(loc_tmp % net->mesh_k != dest_tmp % net->mesh_k)	//loc and dest are not @ the same location within this dimension
							break;
						loc_tmp /= net->mesh_k;
						dest_tmp /= net->mesh_k;
					}

					loc_tmp %= net->mesh_k;
					dest_tmp %= net->mesh_k;

					if(loc_tmp < dest_tmp)	//move right
					{
						chan_id = net->getMeshRightChannelID(loc_mesh, loc, dim);
					}
					else			//move left
					{
						chan_id = net->getMeshLeftChannelID(loc_mesh, loc, dim);
					}

					out_port = net->getChanSrcIx(chan_id);
				}//end of current node isn't a bridge node
			}//end of @ source mesh
		}// end of this is a mesh node
	}//end of the out_port needs to be calculated

#ifdef _FATTREE_MESH_DEBUG_

	cout << "flit " << f->id << " from src " << f->src << " ";
	if(inject)
	{
		cout << "@ inject src ";
	}
	else
	{
		cout << "@ router " << r->GetID() << " ";
	}

	cout << "through out_port " << out_port << " destined for node " << f->dest << endl;

#endif

	outputs->Clear();
	outputs->AddRange(out_port, vcBegin, vcEnd);
}

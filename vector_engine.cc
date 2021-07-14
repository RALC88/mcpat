/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#include "io.h"
#include "parameter.h"
#include "const.h"
#include "basic_circuit.h"
#include <iostream>
#include <algorithm>
#include "XML_Parse.h"
#include <string>
#include <cmath>
#include <assert.h>
#include "vector_engine.h"
//#include "globalvar.h"


VReg::VReg(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 vectordynp(dyn_p_),
 vrf (0),
 exist(exist_)
 {
	/*
	 * processors have separate architectural register files for each thread.
	 * therefore, the bypass buses need to travel across all the register files.
	 */

	if (!exist) return;
	int  data;

	clockRate = vectordynp.clockRate;
	executionTime = vectordynp.executionTime;
	//**********************************VRF***************************************
	data							 = 64;//vectordynp.vector_data_width;
	interface_ip.is_cache			 = false;
	interface_ip.pure_cam            = false;
	interface_ip.pure_ram            = true;
	interface_ip.line_sz             = int(ceil(data/32.0))*4;
	interface_ip.cache_sz            = 128 * 8;//vectordynp.num_VRF_entry*interface_ip.line_sz;
	interface_ip.assoc               = 1;
	interface_ip.nbanks              = 1;
	interface_ip.out_w               = interface_ip.line_sz*8;
	interface_ip.access_mode         = 1;
	interface_ip.throughput          = 1.0/clockRate;
	interface_ip.latency             = 1.0/clockRate;
	interface_ip.obj_func_dyn_energy = 0;
	interface_ip.obj_func_dyn_power  = 0;
	interface_ip.obj_func_leak_power = 0;
	interface_ip.obj_func_cycle_t    = 1;
	interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
	interface_ip.num_rd_ports    = 2*1;//2*vectordynp.vector_peak_issueW;
	interface_ip.num_wr_ports    = 2*1;//vectordynp.vector_peak_issueW;
	interface_ip.num_se_rd_ports = 0;
	vrf = new ArrayST(&interface_ip, "Vector Register File", Core_device, vectordynp.opt_local, vectordynp.vector_ty);
	vrf->area.set_area(vrf->area.get_area()+ vrf->local_result.area*vectordynp.vector_num_pipelines*cdb_overhead);
	area.set_area(area.get_area()+ vrf->local_result.area*vectordynp.vector_num_pipelines*cdb_overhead);
	//area.set_area(area.get_area()*cdb_overhead);
	//output_data_csv(VRF.RF.local_result);
 }

VEXECU::VEXECU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 vectordynp(dyn_p_),
 vector_reg_file(0),
 fp_u(0),
 exeu(0),
 mul(0),
 exist(exist_)
{
	  bool exist_flag = true;
	  if (!exist) return;
	  double fu_height = 0.0;
      clockRate = vectordynp.clockRate;
      executionTime = vectordynp.executionTime;
	  vector_reg_file   = new VReg(XML, ithCore, &interface_ip,vectordynp);
	  exeu  = new FunctionalUnit(XML, ithCore,&interface_ip, vectordynp, ALU);
	  area.set_area(area.get_area()+ exeu->area.get_area() + vector_reg_file->area.get_area());
	  fu_height = exeu->FU_height;
	  if (vectordynp.vector_num_fpus >0)
	  {
		  fp_u  = new FunctionalUnit(XML, ithCore,&interface_ip, vectordynp, FPU);
		  area.set_area(area.get_area()+ fp_u->area.get_area());
	  }
	  if (vectordynp.vector_num_muls >0)
	  {
		  mul   = new FunctionalUnit(XML, ithCore,&interface_ip, vectordynp, MUL);
		  area.set_area(area.get_area()+ mul->area.get_area());
		  fu_height +=  mul->FU_height;
	  }
	  /*
	   * broadcast logic, including int-broadcast; int_tag-broadcast; fp-broadcast; fp_tag-broadcast
	   * integer by pass has two paths and fp has 3 paths.
	   * on the same bus there are multiple tri-state drivers and muxes that go to different components on the same bus
	   */
	  if (XML->sys.Embedded)
	  		{
	  		interface_ip.wt                  =Global_30;
	  		interface_ip.wire_is_mat_type = 0;
	  		interface_ip.wire_os_mat_type = 0;
	  	    interface_ip.throughput       = 1.0/clockRate;
	  	    interface_ip.latency          = 1.0/clockRate;
	  		}
	  	else
	  		{
	  		interface_ip.wt                  =Global;
	  		interface_ip.wire_is_mat_type = 2;//start from semi-global since local wires are already used
	  		interface_ip.wire_os_mat_type = 2;
	  	    interface_ip.throughput       = 10.0/clockRate; //Do not care
	  	    interface_ip.latency          = 10.0/clockRate;
	  		}

	  area.set_area(area.get_area());
}


VectorLane::VectorLane(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 exu(0)
{

 /*
  * initialize, compute and optimize individual components.
  */

  bool exit_flag = true;

  double pipeline_area_per_unit;
//  set_lane_param();

  clockRate = vectordynp.clockRate;
  executionTime = vectordynp.executionTime;
  exu          = new VEXECU     (XML, ithCore, &interface_ip,vectordynp,exit_flag);

  if (exu->exist)
  {
      exu->area.set_area(exu->area.get_area() + pipeline_area_per_unit);
      area.set_area(area.get_area()+exu->area.get_area());
  }

}

VectorEngine::VectorEngine(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_)//,
// lanes(0)
{

 /*
  * initialize, compute and optimize individual components.
  */

  bool exit_flag = false; // bool exit_flag = true; // Checar que es esto

  double pipeline_area_per_unit;
  set_vector_param();

  clockRate = vectordynp.clockRate;
  executionTime = vectordynp.executionTime;

    for (int i = 0;i < numLanes; i++)
    {
          lanes.push_back(new VectorLane(XML,i, &interface_ip));
          lanes[i]->computeEnergy();
          lanes[i]->computeEnergy(false);

          lane.area.set_area(lane.area.get_area() + lanes[i]->area.get_area());
          area.set_area(area.get_area() + lanes[i]->area.get_area());

          //set_pppm(pppm_t,lanes[i]->clockRate, 1, 1, 1);
          //lane.power = lane.power + lanes[i]->power*pppm_t;
          //power = power  + lanes[i]->power*pppm_t;

          //set_pppm(pppm_t,1/lanes[i]->executionTime, 1, 1, 1);
          //lane.rt_power = lane.rt_power + lanes[i]->rt_power*pppm_t;
          //rt_power = rt_power  + lanes[i]->rt_power*pppm_t;
    }
}


void VReg::computeEnergy(bool is_tdp)
{
/*
 * Architecture RF and physical RF cannot be present at the same time.
 * Therefore, the RF stats can only refer to either ARF or PRF;
 * And the same stats can be used for both.
 */
	if (!exist) return;
	if (is_tdp)
    {
    	//init stats for Peak
    	vrf->stats_t.readAc.access  = vectordynp.vector_issueW*2*(vectordynp.ALU_duty_cycle*1.1+
    			(vectordynp.vector_num_muls>0?vectordynp.MUL_duty_cycle:0))*vectordynp.vector_num_pipelines;
    	vrf->stats_t.writeAc.access  = vectordynp.vector_issueW*(vectordynp.ALU_duty_cycle*1.1+
    			(vectordynp.vector_num_muls>0?vectordynp.MUL_duty_cycle:0))*vectordynp.vector_num_pipelines;
    	//Rule of Thumb: about 10% RF related instructions do not need to access ALUs
    	vrf->tdp_stats = vrf->stats_t;
     }
    else
    {
    	//init stats for Runtime Dynamic (RTP)
    	vrf->stats_t.readAc.access  = XML->sys.vector_engine[ithCore].vec_regfile_reads;//TODO: no diff on archi and phy
    	vrf->stats_t.writeAc.access  = XML->sys.vector_engine[ithCore].vec_regfile_writes;
    	vrf->rtp_stats = vrf->stats_t;
    }
	vrf->power_t.reset();
	vrf->power_t.readOp.dynamic  +=  (vrf->stats_t.readAc.access*vrf->local_result.power.readOp.dynamic
			+vrf->stats_t.writeAc.access*vrf->local_result.power.writeOp.dynamic);

	if (is_tdp)
	{
		vrf->power  =  vrf->power_t + ((vectordynp.scheu_ty==ReservationStation) ? (vrf->local_result.power *vectordynp.pppm_lkg_multhread):vrf->local_result.power);
		power	    =  power + (vrf->power);
	}
	else
	{
		vrf->rt_power  =  vrf->power_t + ((vectordynp.scheu_ty==ReservationStation) ? (vrf->local_result.power *vectordynp.pppm_lkg_multhread):vrf->local_result.power);
		rt_power	   =  rt_power + (vrf->power_t);
	}
}


void VReg::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;

	if (is_tdp)
	{	cout << indent_str << "Vector Register File:" << endl;
		cout << indent_str_next << "Area = " << vrf->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << vrf->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? vrf->power.readOp.longer_channel_leakage:vrf->power.readOp.leakage) <<" W" << endl;
		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
				<< (long_channel? vrf->power.readOp.power_gated_with_long_channel_leakage : vrf->power.readOp.power_gated_leakage)  << " W" << endl;
		cout << indent_str_next << "Gate Leakage = " << vrf->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << vrf->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}
	else
	{
		cout << indent_str_next << "VRF    Peak Dynamic = " << vrf->rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "VRF    Subthreshold Leakage = " << vrf->rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "VRF    Gate Leakage = " << vrf->rt_power.readOp.gate_leakage << " W" << endl;
	}
}


void VEXECU::computeEnergy(bool is_tdp)
{
	if (!exist) return;
	double pppm_t[4]    = {1,1,1,1};

	vector_reg_file->computeEnergy(is_tdp);
	exeu->computeEnergy(is_tdp);

	if (vectordynp.vector_num_fpus >0)
	{
		fp_u->computeEnergy(is_tdp);
	}
	if (vectordynp.vector_num_muls >0)
	{
		mul->computeEnergy(is_tdp);
	}

	if (is_tdp)
	{
		if (vectordynp.vector_num_muls >0)
		{
			power      = power + mul->power;
		}
		if (vectordynp.vector_num_fpus>0)
		{
			power      = power + fp_u->power;
		}

		power      = power + vector_reg_file->power + exeu->power;
	}
	else
	{
		if (vectordynp.vector_num_muls >0)
		{
			rt_power      = rt_power + mul->rt_power;
		}

		if (vectordynp.vector_num_fpus>0)
		{
			rt_power      = rt_power + fp_u->rt_power;
		}
		rt_power      = rt_power + vector_reg_file->rt_power + exeu->rt_power;
	}
}

void VEXECU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;

	if (is_tdp)
	{
		cout << indent_str << "Register Files:" << endl;
		cout << indent_str_next << "Area = " << vector_reg_file->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << vector_reg_file->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? vector_reg_file->power.readOp.longer_channel_leakage:vector_reg_file->power.readOp.leakage) <<" W" << endl;
		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
				<< (long_channel? vector_reg_file->power.readOp.power_gated_with_long_channel_leakage : vector_reg_file->power.readOp.power_gated_leakage)  << " W" << endl;
		cout << indent_str_next << "Gate Leakage = " << vector_reg_file->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << vector_reg_file->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
		if (plevel>3){
			vector_reg_file->displayEnergy(indent+4,is_tdp);
		}
		exeu->displayEnergy(indent,is_tdp);
		if (vectordynp.vector_num_fpus>0)
		{
			fp_u->displayEnergy(indent,is_tdp);
		}
		if (vectordynp.vector_num_muls >0)
		{
			mul->displayEnergy(indent,is_tdp);
		}
	}
	else
	{
		cout << indent_str_next << "Register Files    Peak Dynamic = " << vector_reg_file->rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Register Files    Subthreshold Leakage = " << vector_reg_file->rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Register Files    Gate Leakage = " << vector_reg_file->rt_power.readOp.gate_leakage << " W" << endl;
	}

}

void VectorLane::computeEnergy(bool is_tdp)
{
	/*
	 * When computing TDP, power = energy_per_cycle (the value computed in this function) * clock_rate (in the display_energy function)
	 * When computing dyn_power; power = total energy (the value computed in this function) / Total execution time (cycle count / clock rate)
	 */
	//power_point_product_masks
	double pppm_t[4]    = {1,1,1,1};
    double rtp_pipeline_coe;
    double num_units = 1.0;
	if (is_tdp)
	{
		exu->computeEnergy(is_tdp);

		if (exu->exist)
		{
			set_pppm(pppm_t, vectordynp.vector_num_pipelines/num_units*vectordynp.ALU_duty_cycle, vectordynp.vector_num_pipelines/num_units, vectordynp.vector_num_pipelines/num_units, vectordynp.vector_num_pipelines/num_units);
			exu->power = exu->power;
//			cout << "EXE = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
			power     = power + exu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
		}
	}
	else
	{
		exu->computeEnergy(is_tdp);
        num_units = 1.0;
		
		if (exu->exist)
		{
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = vectordynp.pipeline_duty_cycle * vectordynp.ALU_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = vectordynp.pipeline_duty_cycle * vectordynp.ALU_duty_cycle * vectordynp.total_cycles;
			}
			set_pppm(pppm_t, vectordynp.vector_num_pipelines*rtp_pipeline_coe/num_units, vectordynp.vector_num_pipelines/num_units, vectordynp.vector_num_pipelines/num_units, vectordynp.vector_num_pipelines/num_units);
			exu->rt_power = exu->rt_power;
			rt_power     = rt_power  + exu->rt_power;
		}

	}

}

void VectorLane::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;

	if (is_tdp)
	{
		cout << "Vector Lane:" << endl;
		cout << indent_str << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str << "Subthreshold Leakage = "
			<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
		if (power_gating) cout << indent_str << "Subthreshold Leakage with power gating = "
				<< (long_channel? power.readOp.power_gated_with_long_channel_leakage : power.readOp.power_gated_leakage)  << " W" << endl;
		cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
		cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout<<endl;
		if (exu->exist)
		{
			cout << indent_str<< "Execution Unit:" << endl;
			cout << indent_str_next << "Area = " << exu->area.get_area()  *1e-6<< " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? exu->power.readOp.longer_channel_leakage:exu->power.readOp.leakage)   << " W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
					<< (long_channel? exu->power.readOp.power_gated_with_long_channel_leakage : exu->power.readOp.power_gated_leakage)  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << exu->rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
			if (plevel >2){
				exu->displayEnergy(indent+4,plevel,is_tdp);
			}
		}
	}
	else
	{

	}
}


void VectorEngine::computeEnergy(bool is_tdp)
{
	/*
	 * When computing TDP, power = energy_per_cycle (the value computed in this function) * clock_rate (in the display_energy function)
	 * When computing dyn_power; power = total energy (the value computed in this function) / Total execution time (cycle count / clock rate)
	 */
	//power_point_product_masks
	double pppm_t[4];

	for (int i = 0;i < numLanes; i++)
    {
    	pppm_t[i] = 1;
    }

    for (int i = 0;i < numLanes; i++)
    {
    	if (is_tdp)
		{
          set_pppm(pppm_t,lanes[i]->clockRate, 1, 1, 1);
          lane.power = lane.power + lanes[i]->power*pppm_t;
          power = power  + lanes[i]->power*pppm_t;

          set_pppm(pppm_t,1/lanes[i]->executionTime, 1, 1, 1);
          lane.rt_power = lane.rt_power + lanes[i]->rt_power*pppm_t;
          rt_power = rt_power  + lanes[i]->rt_power*pppm_t;
		}
		else
		{
          
        }
    }

}

void VectorEngine::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;


    for (int i = 0;i < numLanes; i++)
    {
    	if (is_tdp)
		{
			cout << "Vector Engine:" << endl;
			cout << indent_str << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
			cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*clockRate << " W" << endl;
			cout << indent_str << "Subthreshold Leakage = "
				<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
			if (power_gating) cout << indent_str << "Subthreshold Leakage with power gating = "
					<< (long_channel? power.readOp.power_gated_with_long_channel_leakage : power.readOp.power_gated_leakage)  << " W" << endl;
			cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
			cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout<<endl;
			if (plevel >2){
				lanes[i]->displayEnergy(indent+4,plevel,is_tdp);
			}
		}
		else
		{

    	}
	}
}

VReg::~VReg(){
	if (!exist) return;
	if(vrf) 	               {delete vrf; vrf = 0;}
	}

VEXECU::~VEXECU(){

	if (!exist) return;
    if(fp_u)                   {delete fp_u;fp_u = 0;}
    if(exeu)                   {delete exeu;exeu = 0;}
    if(mul)                    {delete mul;mul = 0;}
    if(vector_reg_file)        {delete vector_reg_file;vector_reg_file = 0;}
	}

VectorEngine::~VectorEngine(){

//	if(exu) 	               {delete exu; exu = 0;}
	}

void VectorEngine::set_vector_param()
{

	vectordynp.opt_local = XML->sys.core[ithCore].opt_local;
	vectordynp.Embedded = XML->sys.Embedded;
	vectordynp.vector_ty   = (enum Core_type)XML->sys.vector_engine[ithCore].vector_machine_type;
    vectordynp.vector_issueW    = XML->sys.vector_engine[ithCore].vector_issue_width;
    vectordynp.vector_peak_issueW   = XML->sys.vector_engine[ithCore].vector_peak_issue_width;
    vectordynp.vector_commitW       = XML->sys.vector_engine[ithCore].vector_commit_width;
    vectordynp.vector_peak_commitW  = XML->sys.vector_engine[ithCore].vector_peak_issue_width;
    vectordynp.vector_num_fpus      = XML->sys.vector_engine[ithCore].FPU_per_lane;
    vectordynp.vector_num_muls      = XML->sys.vector_engine[ithCore].MUL_per_lane;
    vectordynp.vdd	       = XML->sys.vector_engine[ithCore].vdd;
    vectordynp.power_gating_vcc	   = XML->sys.vector_engine[ithCore].power_gating_vcc;

    vectordynp.vector_num_pipelines= XML->sys.vector_engine[ithCore].pipelines_per_vector_engine[0];

	vectordynp.vector_arch_reg_width  =  int(ceil(log2(XML->sys.vector_engine[ithCore].archi_Regs_VRF_size)));
	vectordynp.num_VRF_entry    = XML->sys.vector_engine[ithCore].archi_Regs_VRF_size;

	vectordynp.pipeline_duty_cycle = XML->sys.vector_engine[ithCore].pipeline_duty_cycle;
	vectordynp.total_cycles        = XML->sys.vector_engine[ithCore].total_cycles;
	vectordynp.busy_cycles         = XML->sys.vector_engine[ithCore].busy_cycles;
	vectordynp.idle_cycles         = XML->sys.vector_engine[ithCore].idle_cycles;

	vectordynp.ALU_duty_cycle = XML->sys.vector_engine[ithCore].ALU_duty_cycle;
	vectordynp.MUL_duty_cycle = XML->sys.vector_engine[ithCore].MUL_duty_cycle;
	vectordynp.FPU_duty_cycle = XML->sys.vector_engine[ithCore].FPU_duty_cycle;


	vectordynp.perThreadState     =  8;
	vectordynp.clockRate          =  XML->sys.core[ithCore].clock_rate;
	vectordynp.clockRate          *= 1e6;
	
	vectordynp.executionTime = XML->sys.total_cycles/vectordynp.clockRate;

	//does not care device types, since all core device types are set at sys. level
	if (vectordynp.vdd > 0)
	{
		interface_ip.specific_hp_vdd = true;
		interface_ip.specific_lop_vdd = true;
		interface_ip.specific_lstp_vdd = true;
		interface_ip.hp_Vdd   = vectordynp.vdd;
		interface_ip.lop_Vdd  = vectordynp.vdd;
		interface_ip.lstp_Vdd = vectordynp.vdd;
	}

	if (vectordynp.power_gating_vcc > -1)
	{
		interface_ip.specific_vcc_min = true;
		interface_ip.user_defined_vcc_min   = vectordynp.power_gating_vcc;
	}
}

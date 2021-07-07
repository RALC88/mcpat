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


// RegFU::RegFU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
// :XML(XML_interface),
//  ithCore(ithCore_),
//  interface_ip(*interface_ip_),
//  coredynp(dyn_p_),
//  IRF (0),
//  FRF (0),
//  exist(exist_)
//  {
// 	/*
// 	 * processors have separate architectural register files for each thread.
// 	 * therefore, the bypass buses need to travel across all the register files.
// 	 */

// 	if (!exist) return;
// 	int  data;

// 	clockRate = coredynp.clockRate;
// 	executionTime = coredynp.executionTime;
// 	//**********************************IRF***************************************
// 	data							 = coredynp.int_data_width;
// 	interface_ip.is_cache			 = false;
// 	interface_ip.pure_cam            = false;
// 	interface_ip.pure_ram            = true;
// 	interface_ip.line_sz             = int(ceil(data/32.0))*4;
// 	interface_ip.cache_sz            = coredynp.num_IRF_entry*interface_ip.line_sz;
// 	interface_ip.assoc               = 1;
// 	interface_ip.nbanks              = 1;
// 	interface_ip.out_w               = interface_ip.line_sz*8;
// 	interface_ip.access_mode         = 1;
// 	interface_ip.throughput          = 1.0/clockRate;
// 	interface_ip.latency             = 1.0/clockRate;
// 	interface_ip.obj_func_dyn_energy = 0;
// 	interface_ip.obj_func_dyn_power  = 0;
// 	interface_ip.obj_func_leak_power = 0;
// 	interface_ip.obj_func_cycle_t    = 1;
// 	interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
// 	interface_ip.num_rd_ports    = 2*coredynp.peak_issueW;
// 	interface_ip.num_wr_ports    = coredynp.peak_issueW;
// 	interface_ip.num_se_rd_ports = 0;
// 	IRF = new ArrayST(&interface_ip, "Integer Register File", Core_device, coredynp.opt_local, coredynp.core_ty);
// 	IRF->area.set_area(IRF->area.get_area()+ IRF->local_result.area*coredynp.num_pipelines*cdb_overhead*((coredynp.scheu_ty==ReservationStation)?XML->sys.core[ithCore].number_hardware_threads:1));
// 	area.set_area(area.get_area()+ IRF->local_result.area*coredynp.num_pipelines*cdb_overhead*((coredynp.scheu_ty==ReservationStation)?XML->sys.core[ithCore].number_hardware_threads:1));
// 	//area.set_area(area.get_area()*cdb_overhead);
// 	//output_data_csv(IRF.RF.local_result);

// 	//**********************************FRF***************************************
// 	data							 = coredynp.fp_data_width;
// 	interface_ip.is_cache			 = false;
// 	interface_ip.pure_cam            = false;
// 	interface_ip.pure_ram            = true;
// 	interface_ip.line_sz             = int(ceil(data/32.0))*4;
// 	interface_ip.cache_sz            = coredynp.num_FRF_entry*interface_ip.line_sz;
// 	interface_ip.assoc               = 1;
// 	interface_ip.nbanks              = 1;
// 	interface_ip.out_w               = interface_ip.line_sz*8;
// 	interface_ip.access_mode         = 1;
// 	interface_ip.throughput          = 1.0/clockRate;
// 	interface_ip.latency             = 1.0/clockRate;
// 	interface_ip.obj_func_dyn_energy = 0;
// 	interface_ip.obj_func_dyn_power  = 0;
// 	interface_ip.obj_func_leak_power = 0;
// 	interface_ip.obj_func_cycle_t    = 1;
// 	interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
// 	interface_ip.num_rd_ports    = 2*XML->sys.core[ithCore].issue_width;
// 	interface_ip.num_wr_ports    = XML->sys.core[ithCore].issue_width;
// 	interface_ip.num_se_rd_ports = 0;
// 	FRF = new ArrayST(&interface_ip, "Floating point Register File", Core_device, coredynp.opt_local, coredynp.core_ty);
// 	FRF->area.set_area(FRF->area.get_area()+ FRF->local_result.area*coredynp.num_fp_pipelines*cdb_overhead*((coredynp.scheu_ty==ReservationStation)?XML->sys.core[ithCore].number_hardware_threads:1));
// 	area.set_area(area.get_area()+ FRF->local_result.area*coredynp.num_fp_pipelines*cdb_overhead*((coredynp.scheu_ty==ReservationStation)?XML->sys.core[ithCore].number_hardware_threads:1));
// 	//area.set_area(area.get_area()*cdb_overhead);
// 	//output_data_csv(FRF.RF.local_result);
// 	int_regfile_height= IRF->local_result.cache_ht*((coredynp.scheu_ty==ReservationStation)?XML->sys.core[ithCore].number_hardware_threads:1)*sqrt(cdb_overhead);
// 	fp_regfile_height = FRF->local_result.cache_ht*((coredynp.scheu_ty==ReservationStation)?XML->sys.core[ithCore].number_hardware_threads:1)*sqrt(cdb_overhead);
//     //since a EXU is associated with each pipeline, the cdb should not have longer length.
// 	if (coredynp.regWindowing)
// 	{
// 		//*********************************REG_WIN************************************
// 		data							 = coredynp.int_data_width; //ECC, and usually 2 regs are transfered together during window shifting.Niagara Mega cell
// 		interface_ip.is_cache			 = false;
// 		interface_ip.pure_cam            = false;
// 		interface_ip.pure_ram            = true;
// 		interface_ip.line_sz             = int(ceil(data/8.0));
// 		interface_ip.cache_sz            = XML->sys.core[ithCore].register_windows_size*IRF->l_ip.cache_sz*XML->sys.core[ithCore].number_hardware_threads;
// 		interface_ip.assoc               = 1;
// 		interface_ip.nbanks              = 1;
// 		interface_ip.out_w               = interface_ip.line_sz*8;
// 		interface_ip.access_mode         = 1;
// 		interface_ip.throughput          = 4.0/clockRate;
// 		interface_ip.latency             = 4.0/clockRate;
// 		interface_ip.obj_func_dyn_energy = 0;
// 		interface_ip.obj_func_dyn_power  = 0;
// 		interface_ip.obj_func_leak_power = 0;
// 		interface_ip.obj_func_cycle_t    = 1;
// 		interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
// 		interface_ip.num_rd_ports    = 0;
// 		interface_ip.num_wr_ports    = 0;
// 		interface_ip.num_se_rd_ports = 0;
// 		RFWIN = new ArrayST(&interface_ip, "RegWindow", Core_device, coredynp.opt_local, coredynp.core_ty);
// 		RFWIN->area.set_area(RFWIN->area.get_area()+ RFWIN->local_result.area*coredynp.num_pipelines);
// 		area.set_area(area.get_area()+ RFWIN->local_result.area*coredynp.num_pipelines);
// 		//output_data_csv(RFWIN.RF.local_result);
// 	}


//  }

// EXECU::EXECU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, double lsq_height_, const CoreDynParam & dyn_p_, bool exist_)
// :XML(XML_interface),
//  ithCore(ithCore_),
//  interface_ip(*interface_ip_),
//  lsq_height(lsq_height_),
//  coredynp(dyn_p_),
//  rfu(0),
//  scheu(0),
//  fp_u(0),
//  exeu(0),
//  mul(0),
//  int_bypass(0),
//  intTagBypass(0),
//  int_mul_bypass(0),
//  intTag_mul_Bypass(0),
//  fp_bypass(0),
//  fpTagBypass(0),
//  exist(exist_)
// {
// 	  bool exist_flag = true;
// 	  if (!exist) return;
// 	  double fu_height = 0.0;
//       clockRate = coredynp.clockRate;
//       executionTime = coredynp.executionTime;
// 	  rfu   = new RegFU(XML, ithCore, &interface_ip,coredynp);
// 	  exeu  = new FunctionalUnit(XML, ithCore,&interface_ip, coredynp, ALU);
// 	  area.set_area(area.get_area()+ exeu->area.get_area() + rfu->area.get_area() +scheu->area.get_area() );
// 	  fu_height = exeu->FU_height;
// 	  if (coredynp.num_fpus >0)
// 	  {
// 		  fp_u  = new FunctionalUnit(XML, ithCore,&interface_ip, coredynp, FPU);
// 		  area.set_area(area.get_area()+ fp_u->area.get_area());
// 	  }
// 	  if (coredynp.num_muls >0)
// 	  {
// 		  mul   = new FunctionalUnit(XML, ithCore,&interface_ip, coredynp, MUL);
// 		  area.set_area(area.get_area()+ mul->area.get_area());
// 		  fu_height +=  mul->FU_height;
// 	  }
// 	  /*
// 	   * broadcast logic, including int-broadcast; int_tag-broadcast; fp-broadcast; fp_tag-broadcast
// 	   * integer by pass has two paths and fp has 3 paths.
// 	   * on the same bus there are multiple tri-state drivers and muxes that go to different components on the same bus
// 	   */
// 	  if (XML->sys.Embedded)
// 	  		{
// 	  		interface_ip.wt                  =Global_30;
// 	  		interface_ip.wire_is_mat_type = 0;
// 	  		interface_ip.wire_os_mat_type = 0;
// 	  	    interface_ip.throughput       = 1.0/clockRate;
// 	  	    interface_ip.latency          = 1.0/clockRate;
// 	  		}
// 	  	else
// 	  		{
// 	  		interface_ip.wt                  =Global;
// 	  		interface_ip.wire_is_mat_type = 2;//start from semi-global since local wires are already used
// 	  		interface_ip.wire_os_mat_type = 2;
// 	  	    interface_ip.throughput       = 10.0/clockRate; //Do not care
// 	  	    interface_ip.latency          = 10.0/clockRate;
// 	  		}

// 	  if (coredynp.core_ty==Inorder)
// 	  {
// 		  int_bypass   = new interconnect("Int Bypass Data", Core_device, 1, 1, int(ceil(XML->sys.machine_bits/32.0)*32),
// 				  rfu->int_regfile_height + exeu->FU_height + lsq_height, &interface_ip, 3,
// 				  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 		  bypass.area.set_area(bypass.area.get_area() + int_bypass->area.get_area());
// 		  intTagBypass = new interconnect("Int Bypass tag" , Core_device, 1, 1, coredynp.perThreadState,
// 				  rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height, &interface_ip, 3,
// 				  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 		  bypass.area.set_area(bypass.area.get_area()  +intTagBypass->area.get_area());

// 		  if (coredynp.num_muls>0)
// 		  {
// 			  int_mul_bypass     = new interconnect("Mul Bypass Data" , Core_device, 1, 1, int(ceil(XML->sys.machine_bits/32.0)*32*1.5),
// 					  rfu->fp_regfile_height + exeu->FU_height + mul->FU_height + lsq_height, &interface_ip, 3,
// 					  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area()  +int_mul_bypass->area.get_area());
// 			  intTag_mul_Bypass  = new interconnect("Mul Bypass tag"  , Core_device, 1, 1, coredynp.perThreadState,
// 					  rfu->fp_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height, &interface_ip, 3,
// 					  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area()  +intTag_mul_Bypass->area.get_area());
// 		  }

// 		  if (coredynp.num_fpus>0)
// 		  {
// 			  fp_bypass    = new interconnect("FP Bypass Data" , Core_device, 1, 1, int(ceil(XML->sys.machine_bits/32.0)*32*1.5),
// 					  rfu->fp_regfile_height + fp_u->FU_height, &interface_ip, 3,
// 					  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area()  +fp_bypass->area.get_area());
// 			  fpTagBypass  = new interconnect("FP Bypass tag"  , Core_device, 1, 1, coredynp.perThreadState,
// 					  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->Iw_height, &interface_ip, 3,
// 					  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area()  +fpTagBypass->area.get_area());
// 		  }
// 	  }
// 	  else
// 	  {//OOO
// 		  if (coredynp.scheu_ty==PhysicalRegFile)
// 		  {
// 			  /* For physical register based OOO,
// 			   * data broadcast interconnects cover across functional units, lsq, inst windows and register files,
// 			   * while tag broadcast interconnects also cover across ROB
// 			   */
// 			  int_bypass   = new interconnect("Int Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
// 					            rfu->int_regfile_height + exeu->FU_height + lsq_height, &interface_ip, 3,
// 								false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area()  +int_bypass->area.get_area());
// 			  intTagBypass = new interconnect("Int Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
// 					            rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
// 								false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area()  +intTagBypass->area.get_area());

// 			  if (coredynp.num_muls>0)
// 			  {
// 				  int_mul_bypass   = new interconnect("Mul Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
// 										rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height, &interface_ip, 3,
// 										false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  intTag_mul_Bypass = new interconnect("Mul Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
// 										rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
// 										false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  bypass.area.set_area(bypass.area.get_area()  +int_mul_bypass->area.get_area());
// 				  bypass.area.set_area(bypass.area.get_area()  +intTag_mul_Bypass->area.get_area());
// 			  }

// 			  if (coredynp.num_fpus>0)
// 			  {
// 				  fp_bypass    = new interconnect("FP Bypass Data" , Core_device, 1, 1, int(ceil(coredynp.fp_data_width)),
// 								  rfu->fp_regfile_height + fp_u->FU_height, &interface_ip, 3,
// 								  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  fpTagBypass  = new interconnect("FP Bypass tag"  , Core_device, 1, 1, coredynp.phy_freg_width,
// 								  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->fp_Iw_height + scheu->ROB_height, &interface_ip, 3,
// 								  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  bypass.area.set_area(bypass.area.get_area()  +fp_bypass->area.get_area());
// 				  bypass.area.set_area(bypass.area.get_area()  +fpTagBypass->area.get_area());
// 			  }
// 		  }
// 		  else
// 		  {
//              /*
//               * In RS based processor both data and tag are broadcast together,
//               * covering functional units, lsq, nst windows, register files, and ROBs
//               */
// 			  int_bypass   = new interconnect("Int Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
// 					            rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height, &interface_ip, 3,
// 								  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  intTagBypass = new interconnect("Int Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
// 					            rfu->int_regfile_height + exeu->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
// 								  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 			  bypass.area.set_area(bypass.area.get_area() +int_bypass->area.get_area());
// 			  bypass.area.set_area(bypass.area.get_area() +intTagBypass->area.get_area());
// 			  if (coredynp.num_muls>0)
// 			  {
// 				  int_mul_bypass   = new interconnect("Mul Bypass Data", Core_device, 1, 1, int(ceil(coredynp.int_data_width)),
// 						            rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height, &interface_ip, 3,
// 									  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  intTag_mul_Bypass = new interconnect("Mul Bypass tag" , Core_device, 1, 1, coredynp.phy_ireg_width,
// 						            rfu->int_regfile_height + exeu->FU_height + mul->FU_height + lsq_height + scheu->Iw_height + scheu->ROB_height , &interface_ip, 3,
// 									  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  bypass.area.set_area(bypass.area.get_area() +int_mul_bypass->area.get_area());
// 				  bypass.area.set_area(bypass.area.get_area() +intTag_mul_Bypass->area.get_area());
// 			  }

// 			  if (coredynp.num_fpus>0)
// 			  {
// 				  fp_bypass    = new interconnect("FP Bypass Data" , Core_device, 1, 1, int(ceil(coredynp.fp_data_width)),
// 						  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->fp_Iw_height + scheu->ROB_height, &interface_ip, 3,
// 						  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  fpTagBypass  = new interconnect("FP Bypass tag"  , Core_device, 1, 1, coredynp.phy_freg_width,
// 						  rfu->fp_regfile_height + fp_u->FU_height + lsq_height + scheu->fp_Iw_height + scheu->ROB_height, &interface_ip, 3,
// 						  false, 1.0, coredynp.opt_local, coredynp.core_ty);
// 				  bypass.area.set_area(bypass.area.get_area() +fp_bypass->area.get_area());
// 				  bypass.area.set_area(bypass.area.get_area() +fpTagBypass->area.get_area());
// 			  }
// 		  }


// 	  }
// 	  area.set_area(area.get_area()+ bypass.area.get_area());
// }


VectorEngine::VectorEngine(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_)//,
// ifu  (0),
// lsu  (0),
// mmu  (0),
// exu  (0),
// rnu  (0),
// corepipe (0),
// undiffCore (0),
// l2cache (0)
{
 /*
  * initialize, compute and optimize individual components.
  */
/*
  bool exit_flag = true;

  double pipeline_area_per_unit;
  //  interface_ip.wire_is_mat_type = 2;
  //  interface_ip.wire_os_mat_type = 2;
  //  interface_ip.wt               =Global_30;
  set_vector_param();

  if (XML->sys.Private_L2)
  {
	  l2cache = new SharedCache(XML,ithCore, &interface_ip);

  }

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  ifu          = new InstFetchU(XML, ithCore, &interface_ip,coredynp,exit_flag);
  lsu          = new LoadStoreU(XML, ithCore, &interface_ip,coredynp,exit_flag);
  mmu          = new MemManU   (XML, ithCore, &interface_ip,coredynp,exit_flag);
  exu          = new EXECU     (XML, ithCore, &interface_ip,lsu->lsq_height, coredynp,exit_flag);
  undiffCore   = new UndiffCore(XML, ithCore, &interface_ip,coredynp,exit_flag);
  if (coredynp.core_ty==OOO)
  {
	  rnu = new RENAMINGU(XML, ithCore, &interface_ip,coredynp);
  }
  corepipe = new Pipeline(&interface_ip,coredynp);

  if (coredynp.core_ty==OOO)
  {
	  pipeline_area_per_unit    = (corepipe->area.get_area()*coredynp.num_pipelines)/5.0;
	  if (rnu->exist)
	  {
		  rnu->area.set_area(rnu->area.get_area() + pipeline_area_per_unit);
	  }
  }
  else {
	  pipeline_area_per_unit    = (corepipe->area.get_area()*coredynp.num_pipelines)/4.0;
  }

  //area.set_area(area.get_area()+ corepipe->area.get_area());
  if (ifu->exist)
  {
	  ifu->area.set_area(ifu->area.get_area() + pipeline_area_per_unit);
	  area.set_area(area.get_area() + ifu->area.get_area());
  }
  if (lsu->exist)
  {
	  lsu->area.set_area(lsu->area.get_area() + pipeline_area_per_unit);
      area.set_area(area.get_area() + lsu->area.get_area());
  }
  if (exu->exist)
  {
	  exu->area.set_area(exu->area.get_area() + pipeline_area_per_unit);
	  area.set_area(area.get_area()+exu->area.get_area());
  }
  if (mmu->exist)
  {
	  mmu->area.set_area(mmu->area.get_area() + pipeline_area_per_unit);
      area.set_area(area.get_area()+mmu->area.get_area());
  }

  if (coredynp.core_ty==OOO)
  {
	  if (rnu->exist)
	  {

		  area.set_area(area.get_area() + rnu->area.get_area());
	  }
  }

  if (undiffCore->exist)
  {
	  area.set_area(area.get_area() + undiffCore->area.get_area());
  }

  if (XML->sys.Private_L2)
  {
	  area.set_area(area.get_area() + l2cache->area.get_area());

  } */
//  //clock power
//  clockNetwork.init_wire_external(is_default, &interface_ip);
//  clockNetwork.clk_area           =area*1.1;//10% of placement overhead. rule of thumb
//  clockNetwork.end_wiring_level   =5;//toplevel metal
//  clockNetwork.start_wiring_level =5;//toplevel metal
//  clockNetwork.num_regs           = corepipe.tot_stage_vector;
//  clockNetwork.optimize_wire();
}


// void RegFU::computeEnergy(bool is_tdp)
// {
// /*
//  * Architecture RF and physical RF cannot be present at the same time.
//  * Therefore, the RF stats can only refer to either ARF or PRF;
//  * And the same stats can be used for both.
//  */
// 	if (!exist) return;
// 	if (is_tdp)
//     {
//     	//init stats for Peak
//     	IRF->stats_t.readAc.access  = coredynp.issueW*2*(coredynp.ALU_duty_cycle*1.1+
//     			(coredynp.num_muls>0?coredynp.MUL_duty_cycle:0))*coredynp.num_pipelines;
//     	IRF->stats_t.writeAc.access  = coredynp.issueW*(coredynp.ALU_duty_cycle*1.1+
//     			(coredynp.num_muls>0?coredynp.MUL_duty_cycle:0))*coredynp.num_pipelines;
//     	//Rule of Thumb: about 10% RF related instructions do not need to access ALUs
//     	IRF->tdp_stats = IRF->stats_t;

//     	FRF->stats_t.readAc.access  = FRF->l_ip.num_rd_ports*coredynp.FPU_duty_cycle*1.05*coredynp.num_fp_pipelines;
//     	FRF->stats_t.writeAc.access  = FRF->l_ip.num_wr_ports*coredynp.FPU_duty_cycle*1.05*coredynp.num_fp_pipelines;
//     	FRF->tdp_stats = FRF->stats_t;
//     	if (coredynp.regWindowing)
//     	{
//         	RFWIN->stats_t.readAc.access  = 0;//0.5*RFWIN->l_ip.num_rw_ports;
//         	RFWIN->stats_t.writeAc.access  = 0;//0.5*RFWIN->l_ip.num_rw_ports;
//         	RFWIN->tdp_stats = RFWIN->stats_t;
//     	}
//      }
//     else
//     {
//     	//init stats for Runtime Dynamic (RTP)
//     	IRF->stats_t.readAc.access  = XML->sys.core[ithCore].int_regfile_reads;//TODO: no diff on archi and phy
//     	IRF->stats_t.writeAc.access  = XML->sys.core[ithCore].int_regfile_writes;
//     	IRF->rtp_stats = IRF->stats_t;

//     	FRF->stats_t.readAc.access  = XML->sys.core[ithCore].float_regfile_reads;
//     	FRF->stats_t.writeAc.access  = XML->sys.core[ithCore].float_regfile_writes;
//     	FRF->rtp_stats = FRF->stats_t;
//     	if (coredynp.regWindowing)
//     	{
//         	RFWIN->stats_t.readAc.access  = XML->sys.core[ithCore].function_calls*16;
//         	RFWIN->stats_t.writeAc.access  = XML->sys.core[ithCore].function_calls*16;
//         	RFWIN->rtp_stats = RFWIN->stats_t;

//         	IRF->stats_t.readAc.access  = XML->sys.core[ithCore].int_regfile_reads +
//         	     XML->sys.core[ithCore].function_calls*16;
//         	IRF->stats_t.writeAc.access  = XML->sys.core[ithCore].int_regfile_writes +
//         	     XML->sys.core[ithCore].function_calls*16;
//         	IRF->rtp_stats = IRF->stats_t;

//         	FRF->stats_t.readAc.access  = XML->sys.core[ithCore].float_regfile_reads +
//    	             XML->sys.core[ithCore].function_calls*16;;
//         	FRF->stats_t.writeAc.access  = XML->sys.core[ithCore].float_regfile_writes+
//    	             XML->sys.core[ithCore].function_calls*16;;
//         	FRF->rtp_stats = FRF->stats_t;
//     	}
//     }
// 	IRF->power_t.reset();
// 	FRF->power_t.reset();
// 	IRF->power_t.readOp.dynamic  +=  (IRF->stats_t.readAc.access*IRF->local_result.power.readOp.dynamic
// 			+IRF->stats_t.writeAc.access*IRF->local_result.power.writeOp.dynamic);
// 	FRF->power_t.readOp.dynamic  +=  (FRF->stats_t.readAc.access*FRF->local_result.power.readOp.dynamic
// 			+FRF->stats_t.writeAc.access*FRF->local_result.power.writeOp.dynamic);
// 	if (coredynp.regWindowing)
// 	{
// 		RFWIN->power_t.reset();
// 		RFWIN->power_t.readOp.dynamic   +=  (RFWIN->stats_t.readAc.access*RFWIN->local_result.power.readOp.dynamic +
// 				RFWIN->stats_t.writeAc.access*RFWIN->local_result.power.writeOp.dynamic);
// 	}

// 	if (is_tdp)
// 	{
// 		IRF->power  =  IRF->power_t + ((coredynp.scheu_ty==ReservationStation) ? (IRF->local_result.power *coredynp.pppm_lkg_multhread):IRF->local_result.power);
// 		FRF->power  =  FRF->power_t + ((coredynp.scheu_ty==ReservationStation) ? (FRF->local_result.power *coredynp.pppm_lkg_multhread):FRF->local_result.power);
// 		power	    =  power + (IRF->power + FRF->power);
// 		if (coredynp.regWindowing)
// 		{
// 			RFWIN->power = RFWIN->power_t + RFWIN->local_result.power *pppm_lkg;
// 			power        = power + RFWIN->power;
// 		}
// 	}
// 	else
// 	{
// 		IRF->rt_power  =  IRF->power_t + ((coredynp.scheu_ty==ReservationStation) ? (IRF->local_result.power *coredynp.pppm_lkg_multhread):IRF->local_result.power);
// 		FRF->rt_power  =  FRF->power_t + ((coredynp.scheu_ty==ReservationStation) ? (FRF->local_result.power *coredynp.pppm_lkg_multhread):FRF->local_result.power);
// 		rt_power	   =  rt_power + (IRF->power_t + FRF->power_t);
// 		if (coredynp.regWindowing)
// 		{
// 			RFWIN->rt_power = RFWIN->power_t + RFWIN->local_result.power *pppm_lkg;
// 			rt_power        = rt_power + RFWIN->rt_power;
// 		}
// 	}
// }


// void RegFU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
// {
// 	if (!exist) return;
// 	string indent_str(indent, ' ');
// 	string indent_str_next(indent+2, ' ');
// 	bool long_channel = XML->sys.longer_channel_device;
// 	bool power_gating = XML->sys.power_gating;

// 	if (is_tdp)
// 	{	cout << indent_str << "Integer RF:" << endl;
// 		cout << indent_str_next << "Area = " << IRF->area.get_area()*1e-6<< " mm^2" << endl;
// 		cout << indent_str_next << "Peak Dynamic = " << IRF->power.readOp.dynamic*clockRate << " W" << endl;
// 		cout << indent_str_next << "Subthreshold Leakage = "
// 			<< (long_channel? IRF->power.readOp.longer_channel_leakage:IRF->power.readOp.leakage) <<" W" << endl;
// 		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
// 				<< (long_channel? IRF->power.readOp.power_gated_with_long_channel_leakage : IRF->power.readOp.power_gated_leakage)  << " W" << endl;
// 		cout << indent_str_next << "Gate Leakage = " << IRF->power.readOp.gate_leakage << " W" << endl;
// 		cout << indent_str_next << "Runtime Dynamic = " << IRF->rt_power.readOp.dynamic/executionTime << " W" << endl;
// 		cout <<endl;
// 		cout << indent_str<< "Floating Point RF:" << endl;
// 		cout << indent_str_next << "Area = " << FRF->area.get_area()*1e-6  << " mm^2" << endl;
// 		cout << indent_str_next << "Peak Dynamic = " << FRF->power.readOp.dynamic*clockRate  << " W" << endl;
// 		cout << indent_str_next << "Subthreshold Leakage = "
// 			<< (long_channel? FRF->power.readOp.longer_channel_leakage:FRF->power.readOp.leakage)  << " W" << endl;
// 		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
// 				<< (long_channel? FRF->power.readOp.power_gated_with_long_channel_leakage : FRF->power.readOp.power_gated_leakage)  << " W" << endl;
// 		cout << indent_str_next << "Gate Leakage = " << FRF->power.readOp.gate_leakage  << " W" << endl;
// 		cout << indent_str_next << "Runtime Dynamic = " << FRF->rt_power.readOp.dynamic/executionTime << " W" << endl;
// 		cout <<endl;
// 		if (coredynp.regWindowing)
// 		{
// 			cout << indent_str << "Register Windows:" << endl;
// 			cout << indent_str_next << "Area = " << RFWIN->area.get_area() *1e-6 << " mm^2" << endl;
// 			cout << indent_str_next << "Peak Dynamic = " << RFWIN->power.readOp.dynamic*clockRate  << " W" << endl;
// 			cout << indent_str_next << "Subthreshold Leakage = "
// 				<< (long_channel? RFWIN->power.readOp.longer_channel_leakage:RFWIN->power.readOp.leakage)  << " W" << endl;
// 			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
// 					<< (long_channel? RFWIN->power.readOp.power_gated_with_long_channel_leakage : RFWIN->power.readOp.power_gated_leakage)  << " W" << endl;
// 			cout << indent_str_next << "Gate Leakage = " << RFWIN->power.readOp.gate_leakage  << " W" << endl;
// 			cout << indent_str_next << "Runtime Dynamic = " << RFWIN->rt_power.readOp.dynamic/executionTime << " W" << endl;
// 			cout <<endl;
// 		}
// 	}
// 	else
// 	{
// 		cout << indent_str_next << "Integer RF    Peak Dynamic = " << IRF->rt_power.readOp.dynamic*clockRate << " W" << endl;
// 		cout << indent_str_next << "Integer RF    Subthreshold Leakage = " << IRF->rt_power.readOp.leakage <<" W" << endl;
// 		cout << indent_str_next << "Integer RF    Gate Leakage = " << IRF->rt_power.readOp.gate_leakage << " W" << endl;
// 		cout << indent_str_next << "Floating Point RF   Peak Dynamic = " << FRF->rt_power.readOp.dynamic*clockRate  << " W" << endl;
// 		cout << indent_str_next << "Floating Point RF   Subthreshold Leakage = " << FRF->rt_power.readOp.leakage  << " W" << endl;
// 		cout << indent_str_next << "Floating Point RF   Gate Leakage = " << FRF->rt_power.readOp.gate_leakage  << " W" << endl;
// 		if (coredynp.regWindowing)
// 		{
// 			cout << indent_str_next << "Register Windows   Peak Dynamic = " << RFWIN->rt_power.readOp.dynamic*clockRate  << " W" << endl;
// 			cout << indent_str_next << "Register Windows   Subthreshold Leakage = " << RFWIN->rt_power.readOp.leakage  << " W" << endl;
// 			cout << indent_str_next << "Register Windows   Gate Leakage = " << RFWIN->rt_power.readOp.gate_leakage  << " W" << endl;
// 		}
// 	}
// }


// void EXECU::computeEnergy(bool is_tdp)
// {
// 	if (!exist) return;
// 	double pppm_t[4]    = {1,1,1,1};
// //	rfu->power.reset();
// //	rfu->rt_power.reset();
// //	scheu->power.reset();
// //	scheu->rt_power.reset();
// //	exeu->power.reset();
// //	exeu->rt_power.reset();

// 	rfu->computeEnergy(is_tdp);
// 	scheu->computeEnergy(is_tdp);
// 	exeu->computeEnergy(is_tdp);
// 	if (coredynp.num_fpus >0)
// 	{
// 		fp_u->computeEnergy(is_tdp);
// 	}
// 	if (coredynp.num_muls >0)
// 	{
// 		mul->computeEnergy(is_tdp);
// 	}

// 	if (is_tdp)
// 	{
// 		set_pppm(pppm_t, 2*coredynp.ALU_cdb_duty_cycle, 2, 2, 2*coredynp.ALU_cdb_duty_cycle);//2 means two source operands needs to be passed for each int instruction.
// 		bypass.power = bypass.power + intTagBypass->power*pppm_t + int_bypass->power*pppm_t;
// 		if (coredynp.num_muls >0)
// 		{
// 			set_pppm(pppm_t, 2*coredynp.MUL_cdb_duty_cycle, 2, 2, 2*coredynp.MUL_cdb_duty_cycle);//2 means two source operands needs to be passed for each int instruction.
// 			bypass.power = bypass.power + intTag_mul_Bypass->power*pppm_t + int_mul_bypass->power*pppm_t;
// 			power      = power + mul->power;
// 		}
// 		if (coredynp.num_fpus>0)
// 		{
// 			set_pppm(pppm_t, 3*coredynp.FPU_cdb_duty_cycle, 3, 3, 3*coredynp.FPU_cdb_duty_cycle);//3 means three source operands needs to be passed for each fp instruction.
// 			bypass.power = bypass.power + fp_bypass->power*pppm_t  + fpTagBypass->power*pppm_t ;
// 			power      = power + fp_u->power;
// 		}

// 		power      = power + rfu->power + exeu->power + bypass.power + scheu->power;
// 	}
// 	else
// 	{
// 		set_pppm(pppm_t, XML->sys.core[ithCore].cdb_alu_accesses, 2, 2, XML->sys.core[ithCore].cdb_alu_accesses);
// 		bypass.rt_power = bypass.rt_power + intTagBypass->power*pppm_t;
// 		bypass.rt_power = bypass.rt_power + int_bypass->power*pppm_t;

// 		if (coredynp.num_muls >0)
// 		{
// 			set_pppm(pppm_t, XML->sys.core[ithCore].cdb_mul_accesses, 2, 2, XML->sys.core[ithCore].cdb_mul_accesses);//2 means two source operands needs to be passed for each int instruction.
// 			bypass.rt_power = bypass.rt_power + intTag_mul_Bypass->power*pppm_t + int_mul_bypass->power*pppm_t;
// 			rt_power      = rt_power + mul->rt_power;
// 		}

// 		if (coredynp.num_fpus>0)
// 		{
// 			set_pppm(pppm_t, XML->sys.core[ithCore].cdb_fpu_accesses, 3, 3, XML->sys.core[ithCore].cdb_fpu_accesses);
// 			bypass.rt_power = bypass.rt_power + fp_bypass->power*pppm_t;
// 			bypass.rt_power = bypass.rt_power + fpTagBypass->power*pppm_t;
// 			rt_power      = rt_power + fp_u->rt_power;
// 		}
// 		rt_power      = rt_power + rfu->rt_power + exeu->rt_power + bypass.rt_power + scheu->rt_power;
// 	}
// }

// void EXECU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
// {
// 	if (!exist) return;
// 	string indent_str(indent, ' ');
// 	string indent_str_next(indent+2, ' ');
// 	bool long_channel = XML->sys.longer_channel_device;
// 	bool power_gating = XML->sys.power_gating;

// //	cout << indent_str_next << "Results Broadcast Bus Area = " << bypass->area.get_area() *1e-6 << " mm^2" << endl;
// 	if (is_tdp)
// 	{
// 		cout << indent_str << "Register Files:" << endl;
// 		cout << indent_str_next << "Area = " << rfu->area.get_area()*1e-6<< " mm^2" << endl;
// 		cout << indent_str_next << "Peak Dynamic = " << rfu->power.readOp.dynamic*clockRate << " W" << endl;
// 		cout << indent_str_next << "Subthreshold Leakage = "
// 			<< (long_channel? rfu->power.readOp.longer_channel_leakage:rfu->power.readOp.leakage) <<" W" << endl;
// 		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
// 				<< (long_channel? rfu->power.readOp.power_gated_with_long_channel_leakage : rfu->power.readOp.power_gated_leakage)  << " W" << endl;
// 		cout << indent_str_next << "Gate Leakage = " << rfu->power.readOp.gate_leakage << " W" << endl;
// 		cout << indent_str_next << "Runtime Dynamic = " << rfu->rt_power.readOp.dynamic/executionTime << " W" << endl;
// 		cout <<endl;
// 		if (plevel>3){
// 			rfu->displayEnergy(indent+4,is_tdp);
// 		}
// 		cout << indent_str << "Instruction Scheduler:" << endl;
// 		cout << indent_str_next << "Area = " << scheu->area.get_area()*1e-6  << " mm^2" << endl;
// 		cout << indent_str_next << "Peak Dynamic = " << scheu->power.readOp.dynamic*clockRate  << " W" << endl;
// 		cout << indent_str_next << "Subthreshold Leakage = "
// 			<< (long_channel? scheu->power.readOp.longer_channel_leakage:scheu->power.readOp.leakage)  << " W" << endl;
// 		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
// 				<< (long_channel? scheu->power.readOp.power_gated_with_long_channel_leakage : scheu->power.readOp.power_gated_leakage)  << " W" << endl;
// 		cout << indent_str_next << "Gate Leakage = " << scheu->power.readOp.gate_leakage  << " W" << endl;
// 		cout << indent_str_next << "Runtime Dynamic = " << scheu->rt_power.readOp.dynamic/executionTime << " W" << endl;
// 		cout <<endl;
// 		if (plevel>3){
// 			scheu->displayEnergy(indent+4,is_tdp);
// 		}
// 		exeu->displayEnergy(indent,is_tdp);
// 		if (coredynp.num_fpus>0)
// 		{
// 			fp_u->displayEnergy(indent,is_tdp);
// 		}
// 		if (coredynp.num_muls >0)
// 		{
// 			mul->displayEnergy(indent,is_tdp);
// 		}
// 		cout << indent_str << "Results Broadcast Bus:" << endl;
// 		cout << indent_str_next << "Area Overhead = " << bypass.area.get_area()*1e-6  << " mm^2" << endl;
// 		cout << indent_str_next << "Peak Dynamic = " << bypass.power.readOp.dynamic*clockRate  << " W" << endl;
// 		cout << indent_str_next << "Subthreshold Leakage = "
// 			<< (long_channel? bypass.power.readOp.longer_channel_leakage:bypass.power.readOp.leakage ) << " W" << endl;
// 		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
// 				<< (long_channel? bypass.power.readOp.power_gated_with_long_channel_leakage : bypass.power.readOp.power_gated_leakage)  << " W" << endl;
// 		cout << indent_str_next << "Gate Leakage = " << bypass.power.readOp.gate_leakage  << " W" << endl;
// 		cout << indent_str_next << "Runtime Dynamic = " << bypass.rt_power.readOp.dynamic/executionTime << " W" << endl;
// 		cout <<endl;
// 	}
// 	else
// 	{
// 		cout << indent_str_next << "Register Files    Peak Dynamic = " << rfu->rt_power.readOp.dynamic*clockRate << " W" << endl;
// 		cout << indent_str_next << "Register Files    Subthreshold Leakage = " << rfu->rt_power.readOp.leakage <<" W" << endl;
// 		cout << indent_str_next << "Register Files    Gate Leakage = " << rfu->rt_power.readOp.gate_leakage << " W" << endl;
// 		cout << indent_str_next << "Instruction Sheduler   Peak Dynamic = " << scheu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
// 		cout << indent_str_next << "Instruction Sheduler   Subthreshold Leakage = " << scheu->rt_power.readOp.leakage  << " W" << endl;
// 		cout << indent_str_next << "Instruction Sheduler   Gate Leakage = " << scheu->rt_power.readOp.gate_leakage  << " W" << endl;
// 		cout << indent_str_next << "Results Broadcast Bus   Peak Dynamic = " << bypass.rt_power.readOp.dynamic*clockRate  << " W" << endl;
// 		cout << indent_str_next << "Results Broadcast Bus   Subthreshold Leakage = " << bypass.rt_power.readOp.leakage  << " W" << endl;
// 		cout << indent_str_next << "Results Broadcast Bus   Gate Leakage = " << bypass.rt_power.readOp.gate_leakage  << " W" << endl;
// 	}

// }

void VectorEngine::computeEnergy(bool is_tdp)
{
	/*
	 * When computing TDP, power = energy_per_cycle (the value computed in this function) * clock_rate (in the display_energy function)
	 * When computing dyn_power; power = total energy (the value computed in this function) / Total execution time (cycle count / clock rate)
	 */
	//power_point_product_masks
/*	double pppm_t[4]    = {1,1,1,1};
    double rtp_pipeline_coe;
    double num_units = 4.0;
	if (is_tdp)
	{
		ifu->computeEnergy(is_tdp);
		lsu->computeEnergy(is_tdp);
		mmu->computeEnergy(is_tdp);
		exu->computeEnergy(is_tdp);

		if (coredynp.core_ty==OOO)
		{
			num_units = 5.0;
			rnu->computeEnergy(is_tdp);
			set_pppm(pppm_t, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);//User need to feed a duty cycle to improve accuracy
			if (rnu->exist)
			{
				rnu->power = rnu->power + corepipe->power*pppm_t;
				power     = power + rnu->power;
			}
		}

		if (ifu->exist)
		{
			set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.IFU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
//			cout << "IFU = " << ifu->power.readOp.dynamic*clockRate  << " W" << endl;
			ifu->power = ifu->power + corepipe->power*pppm_t;
//			cout << "IFU = " << ifu->power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << "1/4 pipe = " << corepipe->power.readOp.dynamic*clockRate/num_units  << " W" << endl;
			power     = power + ifu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
		}
		if (lsu->exist)
		{
			set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.LSU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			lsu->power = lsu->power + corepipe->power*pppm_t;
//			cout << "LSU = " << lsu->power.readOp.dynamic*clockRate  << " W" << endl;
			power     = power + lsu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
		}
		if (exu->exist)
		{
			set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.ALU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			exu->power = exu->power + corepipe->power*pppm_t;
//			cout << "EXE = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
			power     = power + exu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
		}
		if (mmu->exist)
		{
			set_pppm(pppm_t, coredynp.num_pipelines/num_units*(0.5+0.5*coredynp.LSU_duty_cycle), coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			mmu->power = mmu->power + corepipe->power*pppm_t;
//			cout << "MMU = " << mmu->power.readOp.dynamic*clockRate  << " W" << endl;
			power     = power +  mmu->power;
//			cout << "core = " << power.readOp.dynamic*clockRate  << " W" << endl;
		}

		power     = power +  undiffCore->power;

		if (XML->sys.Private_L2)
		{

			l2cache->computeEnergy(is_tdp);
			set_pppm(pppm_t,l2cache->cachep.clockRate/clockRate, 1,1,1);
			//l2cache->power = l2cache->power*pppm_t;
			power = power  + l2cache->power*pppm_t;
		}

	}
	else
	{
		ifu->computeEnergy(is_tdp);
		lsu->computeEnergy(is_tdp);
		mmu->computeEnergy(is_tdp);
		exu->computeEnergy(is_tdp);

		if (coredynp.core_ty==OOO)
		{
			num_units = 5.0;
			rnu->computeEnergy(is_tdp);
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.total_cycles;
			}
        	set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			if (rnu->exist)
			{
        	rnu->rt_power = rnu->rt_power + corepipe->power*pppm_t;

			rt_power      = rt_power + rnu->rt_power;
			}
		}
		else
		{
			num_units = 4.0;
		}

		if (ifu->exist)
		{
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.IFU_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.IFU_duty_cycle * coredynp.total_cycles;
			}
			set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			ifu->rt_power = ifu->rt_power + corepipe->power*pppm_t;
			rt_power     = rt_power + ifu->rt_power ;
		}
		if (lsu->exist)
		{
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.LSU_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.LSU_duty_cycle * coredynp.total_cycles;
			}
			set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);

			lsu->rt_power = lsu->rt_power + corepipe->power*pppm_t;
			rt_power     = rt_power  + lsu->rt_power;
		}
		if (exu->exist)
		{
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.ALU_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.ALU_duty_cycle * coredynp.total_cycles;
			}
			set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			exu->rt_power = exu->rt_power + corepipe->power*pppm_t;
			rt_power     = rt_power  + exu->rt_power;
		}
		if (mmu->exist)
		{
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * (0.5+0.5*coredynp.LSU_duty_cycle) * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * (0.5+0.5*coredynp.LSU_duty_cycle) * coredynp.total_cycles;
			}
			set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
			mmu->rt_power = mmu->rt_power + corepipe->power*pppm_t;
			rt_power     = rt_power +  mmu->rt_power ;

		}

		rt_power     = rt_power +  undiffCore->power;
//		cout << "EXE = " << exu->power.readOp.dynamic*clockRate  << " W" << endl;
		if (XML->sys.Private_L2)
		{

			l2cache->computeEnergy(is_tdp);
			//set_pppm(pppm_t,1/l2cache->cachep.executionTime, 1,1,1);
			//l2cache->rt_power = l2cache->rt_power*pppm_t;
			rt_power = rt_power  + l2cache->rt_power;
		}
	}
*/
}

void VectorEngine::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	/*
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;

	if (is_tdp)
	{
		cout << "Core:" << endl;
		cout << indent_str << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str << "Subthreshold Leakage = "
			<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
		if (power_gating) cout << indent_str << "Subthreshold Leakage with power gating = "
				<< (long_channel? power.readOp.power_gated_with_long_channel_leakage : power.readOp.power_gated_leakage)  << " W" << endl;
		cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
		cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout<<endl;
		if (ifu->exist)
		{
			cout << indent_str << "Instruction Fetch Unit:" << endl;
			cout << indent_str_next << "Area = " << ifu->area.get_area()*1e-6<< " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << ifu->power.readOp.dynamic*clockRate << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? ifu->power.readOp.longer_channel_leakage:ifu->power.readOp.leakage) <<" W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
					<< (long_channel? ifu->power.readOp.power_gated_with_long_channel_leakage : ifu->power.readOp.power_gated_leakage)  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << ifu->power.readOp.gate_leakage << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << ifu->rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
			if (plevel >2){
				ifu->displayEnergy(indent+4,plevel,is_tdp);
			}
		}
		if (coredynp.core_ty==OOO)
		{
			if (rnu->exist)
			{
				cout << indent_str<< "Renaming Unit:" << endl;
				cout << indent_str_next << "Area = " << rnu->area.get_area()*1e-6  << " mm^2" << endl;
				cout << indent_str_next << "Peak Dynamic = " << rnu->power.readOp.dynamic*clockRate  << " W" << endl;
				cout << indent_str_next << "Subthreshold Leakage = "
					<< (long_channel? rnu->power.readOp.longer_channel_leakage:rnu->power.readOp.leakage)  << " W" << endl;
				if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
						<< (long_channel? rnu->power.readOp.power_gated_with_long_channel_leakage : rnu->power.readOp.power_gated_leakage)  << " W" << endl;
				cout << indent_str_next << "Gate Leakage = " << rnu->power.readOp.gate_leakage  << " W" << endl;
				cout << indent_str_next << "Runtime Dynamic = " << rnu->rt_power.readOp.dynamic/executionTime << " W" << endl;
				cout <<endl;
				if (plevel >2){
					rnu->displayEnergy(indent+4,plevel,is_tdp);
				}
			}

		}
		if (lsu->exist)
		{
			cout << indent_str<< "Load Store Unit:" << endl;
			cout << indent_str_next << "Area = " << lsu->area.get_area()*1e-6  << " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << lsu->power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? lsu->power.readOp.longer_channel_leakage:lsu->power.readOp.leakage ) << " W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
					<< (long_channel? lsu->power.readOp.power_gated_with_long_channel_leakage : lsu->power.readOp.power_gated_leakage)  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << lsu->power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << lsu->rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
			if (plevel >2){
				lsu->displayEnergy(indent+4,plevel,is_tdp);
			}
		}
		if (mmu->exist)
		{
			cout << indent_str<< "Memory Management Unit:" << endl;
			cout << indent_str_next << "Area = " << mmu->area.get_area() *1e-6 << " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << mmu->power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? mmu->power.readOp.longer_channel_leakage:mmu->power.readOp.leakage)   << " W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
					<< (long_channel? mmu->power.readOp.power_gated_with_long_channel_leakage : mmu->power.readOp.power_gated_leakage)  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << mmu->rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
			if (plevel >2){
				mmu->displayEnergy(indent+4,plevel,is_tdp);
			}
		}
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
//		if (plevel >2)
//		{
//			if (undiffCore->exist)
//			{
//				cout << indent_str << "Undifferentiated Core" << endl;
//				cout << indent_str_next << "Area = " << undiffCore->area.get_area()*1e-6<< " mm^2" << endl;
//				cout << indent_str_next << "Peak Dynamic = " << undiffCore->power.readOp.dynamic*clockRate << " W" << endl;
////				cout << indent_str_next << "Subthreshold Leakage = " << undiffCore->power.readOp.leakage <<" W" << endl;
//				cout << indent_str_next << "Subthreshold Leakage = "
//								<< (long_channel? undiffCore->power.readOp.longer_channel_leakage:undiffCore->power.readOp.leakage)   << " W" << endl;
//				cout << indent_str_next << "Gate Leakage = " << undiffCore->power.readOp.gate_leakage << " W" << endl;
//				//		cout << indent_str_next << "Runtime Dynamic = " << undiffCore->rt_power.readOp.dynamic/executionTime << " W" << endl;
//				cout <<endl;
//			}
//		}
		if (XML->sys.Private_L2)
		{

			l2cache->displayEnergy(4,is_tdp);
		}

	}
	else
	{
//		cout << indent_str_next << "Instruction Fetch Unit    Peak Dynamic = " << ifu->rt_power.readOp.dynamic*clockRate << " W" << endl;
//		cout << indent_str_next << "Instruction Fetch Unit    Subthreshold Leakage = " << ifu->rt_power.readOp.leakage <<" W" << endl;
//		cout << indent_str_next << "Instruction Fetch Unit    Gate Leakage = " << ifu->rt_power.readOp.gate_leakage << " W" << endl;
//		cout << indent_str_next << "Load Store Unit   Peak Dynamic = " << lsu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Load Store Unit   Subthreshold Leakage = " << lsu->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Load Store Unit   Gate Leakage = " << lsu->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Memory Management Unit   Peak Dynamic = " << mmu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Memory Management Unit   Subthreshold Leakage = " << mmu->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Memory Management Unit   Gate Leakage = " << mmu->rt_power.readOp.gate_leakage  << " W" << endl;
//		cout << indent_str_next << "Execution Unit   Peak Dynamic = " << exu->rt_power.readOp.dynamic*clockRate  << " W" << endl;
//		cout << indent_str_next << "Execution Unit   Subthreshold Leakage = " << exu->rt_power.readOp.leakage  << " W" << endl;
//		cout << indent_str_next << "Execution Unit   Gate Leakage = " << exu->rt_power.readOp.gate_leakage  << " W" << endl;
	}
	*/
}
/*
RegFU ::~RegFU(){

	if (!exist) return;
	if(IRF) 	               {delete IRF; IRF = 0;}
    if(FRF) 	               {delete FRF; FRF = 0;}
    if(RFWIN) 	               {delete RFWIN; RFWIN = 0;}
	}

EXECU ::~EXECU(){

	if (!exist) return;
	if(int_bypass) 	           {delete int_bypass; int_bypass = 0;}
    if(intTagBypass) 	       {delete intTagBypass; intTagBypass =0;}
    if(int_mul_bypass) 	       {delete int_mul_bypass; int_mul_bypass = 0;}
    if(intTag_mul_Bypass) 	   {delete intTag_mul_Bypass; intTag_mul_Bypass =0;}
    if(fp_bypass) 	           {delete fp_bypass;fp_bypass = 0;}
    if(fpTagBypass) 	       {delete fpTagBypass;fpTagBypass = 0;}
    if(fp_u)                   {delete fp_u;fp_u = 0;}
    if(exeu)                   {delete exeu;exeu = 0;}
    if(mul)                    {delete mul;mul = 0;}
    if(rfu)                    {delete rfu;rfu = 0;}
	if(scheu) 	               {delete scheu; scheu = 0;}
	}
*/
VectorEngine ::~VectorEngine(){

//	if(exu) 	               {delete exu; exu = 0;}
	}

void VectorEngine::set_vector_param()
{

	coredynp.opt_local = XML->sys.core[ithCore].opt_local;
	coredynp.x86 = XML->sys.core[ithCore].x86;
	coredynp.Embedded = XML->sys.Embedded;
	coredynp.core_ty   = (enum Core_type)XML->sys.core[ithCore].machine_type;
	coredynp.rm_ty     = (enum Renaming_type)XML->sys.core[ithCore].rename_scheme;
    coredynp.fetchW    = XML->sys.core[ithCore].fetch_width;
    coredynp.decodeW   = XML->sys.core[ithCore].decode_width;
    coredynp.issueW    = XML->sys.core[ithCore].issue_width;
    coredynp.peak_issueW   = XML->sys.core[ithCore].peak_issue_width;
    coredynp.commitW       = XML->sys.core[ithCore].commit_width;
    coredynp.peak_commitW  = XML->sys.core[ithCore].peak_issue_width;
    coredynp.predictionW   = XML->sys.core[ithCore].prediction_width;
    coredynp.fp_issueW     = XML->sys.core[ithCore].fp_issue_width;
    coredynp.fp_decodeW    = XML->sys.core[ithCore].fp_issue_width;
    coredynp.num_alus      = XML->sys.core[ithCore].ALU_per_core;
    coredynp.num_fpus      = XML->sys.core[ithCore].FPU_per_core;
    coredynp.num_muls      = XML->sys.core[ithCore].MUL_per_core;
    coredynp.vdd	       = XML->sys.core[ithCore].vdd;
    coredynp.power_gating_vcc	       = XML->sys.core[ithCore].power_gating_vcc;



    coredynp.num_hthreads	     = XML->sys.core[ithCore].number_hardware_threads;
    coredynp.multithreaded       = coredynp.num_hthreads>1? true:false;
    coredynp.hthread_width       =  int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads)));
    coredynp.instruction_length  = XML->sys.core[ithCore].instruction_length;
    coredynp.pc_width            = XML->sys.virtual_address_width;

   	coredynp.opcode_length       = XML->sys.core[ithCore].opcode_width;
    coredynp.micro_opcode_length = XML->sys.core[ithCore].micro_opcode_width;
    coredynp.num_pipelines       = XML->sys.core[ithCore].pipelines_per_core[0];
    coredynp.pipeline_stages     = XML->sys.core[ithCore].pipeline_depth[0];
    coredynp.num_fp_pipelines    = XML->sys.core[ithCore].pipelines_per_core[1];
    coredynp.fp_pipeline_stages  = XML->sys.core[ithCore].pipeline_depth[1];
    coredynp.int_data_width      = int(ceil(XML->sys.machine_bits/32.0))*32;
    coredynp.fp_data_width       = coredynp.int_data_width;
    coredynp.v_address_width     = XML->sys.virtual_address_width;
    coredynp.p_address_width     = XML->sys.physical_address_width;

	coredynp.scheu_ty         = (enum Scheduler_type)XML->sys.core[ithCore].instruction_window_scheme;
	coredynp.arch_ireg_width  =  int(ceil(log2(XML->sys.core[ithCore].archi_Regs_IRF_size)));
	coredynp.arch_freg_width  =  int(ceil(log2(XML->sys.core[ithCore].archi_Regs_FRF_size)));
	coredynp.num_IRF_entry    = XML->sys.core[ithCore].archi_Regs_IRF_size;
	coredynp.num_FRF_entry    = XML->sys.core[ithCore].archi_Regs_FRF_size;
	coredynp.pipeline_duty_cycle = XML->sys.core[ithCore].pipeline_duty_cycle;
	coredynp.total_cycles        = XML->sys.core[ithCore].total_cycles;
	coredynp.busy_cycles         = XML->sys.core[ithCore].busy_cycles;
	coredynp.idle_cycles         = XML->sys.core[ithCore].idle_cycles;

	//Max power duty cycle for peak power estimation
//	if (coredynp.core_ty==OOO)
//	{
//		coredynp.IFU_duty_cycle = 1;
//		coredynp.LSU_duty_cycle = 1;
//		coredynp.MemManU_I_duty_cycle =1;
//		coredynp.MemManU_D_duty_cycle =1;
//		coredynp.ALU_duty_cycle =1;
//		coredynp.MUL_duty_cycle =1;
//		coredynp.FPU_duty_cycle =1;
//		coredynp.ALU_cdb_duty_cycle =1;
//		coredynp.MUL_cdb_duty_cycle =1;
//		coredynp.FPU_cdb_duty_cycle =1;
//	}
//	else
//	{
		coredynp.IFU_duty_cycle = XML->sys.core[ithCore].IFU_duty_cycle;
		coredynp.BR_duty_cycle = XML->sys.core[ithCore].BR_duty_cycle;
		coredynp.LSU_duty_cycle = XML->sys.core[ithCore].LSU_duty_cycle;
		coredynp.MemManU_I_duty_cycle = XML->sys.core[ithCore].MemManU_I_duty_cycle;
		coredynp.MemManU_D_duty_cycle = XML->sys.core[ithCore].MemManU_D_duty_cycle;
		coredynp.ALU_duty_cycle = XML->sys.core[ithCore].ALU_duty_cycle;
		coredynp.MUL_duty_cycle = XML->sys.core[ithCore].MUL_duty_cycle;
		coredynp.FPU_duty_cycle = XML->sys.core[ithCore].FPU_duty_cycle;
		coredynp.ALU_cdb_duty_cycle = XML->sys.core[ithCore].ALU_cdb_duty_cycle;
		coredynp.MUL_cdb_duty_cycle = XML->sys.core[ithCore].MUL_cdb_duty_cycle;
		coredynp.FPU_cdb_duty_cycle = XML->sys.core[ithCore].FPU_cdb_duty_cycle;
//	}


	if (!((coredynp.core_ty==OOO)||(coredynp.core_ty==Inorder)))
	{
		cout<<"Invalid Core Type"<<endl;
		exit(0);
	}
//	if (coredynp.core_ty==OOO)
//	{
//		cout<<"OOO processor models are being updated and will be available in next release"<<endl;
//		exit(0);
//	}
	if (!((coredynp.scheu_ty==PhysicalRegFile)||(coredynp.scheu_ty==ReservationStation)))
	{
		cout<<"Invalid OOO Scheduler Type"<<endl;
		exit(0);
	}

	if (!((coredynp.rm_ty ==RAMbased)||(coredynp.rm_ty ==CAMbased)))
	{
		cout<<"Invalid OOO Renaming Type"<<endl;
		exit(0);
	}

	if (coredynp.core_ty==OOO)
	{
		if (coredynp.scheu_ty==PhysicalRegFile)
		{
			coredynp.phy_ireg_width  =  int(ceil(log2(XML->sys.core[ithCore].phy_Regs_IRF_size)));
			coredynp.phy_freg_width  =  int(ceil(log2(XML->sys.core[ithCore].phy_Regs_FRF_size)));
			coredynp.num_ifreelist_entries = coredynp.num_IRF_entry  = XML->sys.core[ithCore].phy_Regs_IRF_size;
			coredynp.num_ffreelist_entries = coredynp.num_FRF_entry  = XML->sys.core[ithCore].phy_Regs_FRF_size;
		}
		else if (coredynp.scheu_ty==ReservationStation)
		{//ROB serves as Phy RF in RS based OOO
			coredynp.phy_ireg_width  =  int(ceil(log2(XML->sys.core[ithCore].ROB_size)));
			coredynp.phy_freg_width  =  int(ceil(log2(XML->sys.core[ithCore].ROB_size)));
			coredynp.num_ifreelist_entries = XML->sys.core[ithCore].ROB_size;
			coredynp.num_ffreelist_entries = XML->sys.core[ithCore].ROB_size;

		}

	}

	int GC_count=XML->sys.core[ithCore].checkpoint_depth;//best check pointing entries for a 4~8 issue OOO should be 8~48;See TR for reference.
	if (coredynp.rm_ty ==RAMbased)
	{
		coredynp.globalCheckpoint   =  GC_count > 4 ? 4 : GC_count; //RAM-based RAT cannot have more than 4 GCs; see "a power-aware hybrid ram-cam renaming mechanism for fast recovery"
	}
	else if(coredynp.rm_ty ==CAMbased)
	{
		coredynp.globalCheckpoint   =  GC_count < 1 ? 1 : GC_count;
	}

	coredynp.perThreadState     =  8;
	coredynp.instruction_length = 32;
	coredynp.clockRate          =  XML->sys.core[ithCore].clock_rate;
	coredynp.clockRate          *= 1e6;
	coredynp.regWindowing= (XML->sys.core[ithCore].register_windows_size>0&&coredynp.core_ty==Inorder)?true:false;
	coredynp.executionTime = XML->sys.total_cycles/coredynp.clockRate;
	set_pppm(coredynp.pppm_lkg_multhread, 0, coredynp.num_hthreads, coredynp.num_hthreads, 0);

	//does not care device types, since all core device types are set at sys. level
	if (coredynp.vdd > 0)
	{
		interface_ip.specific_hp_vdd = true;
		interface_ip.specific_lop_vdd = true;
		interface_ip.specific_lstp_vdd = true;
		interface_ip.hp_Vdd   = coredynp.vdd;
		interface_ip.lop_Vdd  = coredynp.vdd;
		interface_ip.lstp_Vdd = coredynp.vdd;
	}

	if (coredynp.power_gating_vcc > -1)
	{
		interface_ip.specific_vcc_min = true;
		interface_ip.user_defined_vcc_min   = coredynp.power_gating_vcc;
	}
}

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


#ifndef VECTOR_ENGINE_H_
#define VECTOR_ENGINE_

#include "XML_Parse.h"
#include "logic.h"
#include "parameter.h"
#include "array.h"
#include "interconnect.h"
#include "basic_components.h"
#include "sharedcache.h"

class VReg :public Component {
  public:

	ParseXML *XML;
	int  ithCore;
	InputParameter interface_ip;
	CoreDynParam  vectordynp;
	double clockRate,executionTime;
	double scktRatio, chip_PR_overhead, macro_PR_overhead;
	double int_regfile_height, fp_regfile_height;
	ArrayST * vrf;
	bool exist;

	VReg(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
	~VReg();
};

class VectorLane :public Component {
  public:

    ParseXML *XML;
    int  ithCore;
    InputParameter interface_ip;
    double clockRate,executionTime;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    CoreDynParam  vectordynp;

    VReg           * vector_reg_file;
    FunctionalUnit * fp_u;
    FunctionalUnit * exeu;
    FunctionalUnit * mul;

    bool exist;

    VectorLane(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_=true);
//    void set_lane_param();
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
    ~VectorLane();
};

class VectorEngine :public Component {
  public:

	ParseXML *XML;
	int  ithCore;
	InputParameter interface_ip;
	double clockRate,executionTime;
	double scktRatio, chip_PR_overhead, macro_PR_overhead;
	vector<VectorLane *> lanes;
    CoreDynParam  vectordynp;
    //full_decoder 	inst_decoder;
    //clock_network	clockNetwork;
    Component lane;
    int numLanes;
	VectorEngine(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_);
	void set_vector_param();
	void computeEnergy(bool is_tdp=true);
	void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
	~VectorEngine();
};

#endif /* VECTOR_ENGINE_H */

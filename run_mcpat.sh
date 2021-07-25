######################################################################################################
# Research Platform for RISC-V SIMD Architectures  v1.0"
# Cristóbal Ramírez Lazo"
# BSC - Barcelona Supercomputing Center"
######################################################################################################
# Example : source run_mcpat blackscholes RVA
######################################################################################################
application=$1 	#
model=$2 			# RVA, native , RG

dir="ProcessorDescriptionFiles/thesis_conf/${application}/${model}"

if [ $2 == "RG" ]; then
	echo "${application},  ${model} configs"
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl16.xml -print_level 5 > ${dir}/mvl16.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl32.xml -print_level 5 > ${dir}/mvl32.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl64.xml -print_level 5 > ${dir}/mvl64.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl128.xml -print_level 5 > ${dir}/mvl128.txt &
else
	echo "${application},  ${model} configs"
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl16.xml -print_level 5 > ${dir}/mvl16.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl32.xml -print_level 5 > ${dir}/mvl32.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl48.xml -print_level 5 > ${dir}/mvl48.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl64.xml -print_level 5 > ${dir}/mvl64.txt &
	nohup ./mcpat -infile ${dir}/Lagarto_22nm_mvl128.xml -print_level 5 > ${dir}/mvl128.txt &
fi
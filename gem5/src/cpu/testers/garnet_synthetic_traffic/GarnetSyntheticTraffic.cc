/*
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
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
 *
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Tushar Krishna
 */

#include "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTraffic.hh"

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <cassert>
#include <sstream>  


#include "base/logging.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "debug/GarnetSyntheticTraffic.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

// CPU satus
# define IDLE               (int(0))
# define WORKIING           (int(1)) 
# define FINISH             (int(2)) 
// CPU working status
# define WORK_WAIT          (int(3))
# define WORK_CAL           (int(4))
# define WORK_SEND          (int(5))
# define WORK_IDLE          (int(6))

# define Traffic_output_file    (std::string("./../output_info/lenet_ga_16/"))
# define Task_folder        (std::string("./../task/lenet_ga_16/"))
# define node2ni            (std::string("./../recv/node2ni/"))
# define recv_pac_file      (std::string("./../recv/packet_num/"))
# define Traffic_node_num   (int(16))
# define FINISH_PIC         (int(0))
using namespace std;

int TESTER_NETWORK=0;
# define start_cycle        (int(1))
# define node0              (int(18))

int start_node[10] = {18, 17, 2, 11, 20, 5, 10, 7, 8, 9};
# define start_node_num (int(0))


//layer1_node [6,18,31,33,29,17,11]
//vector<int> w(100,1);
//vector<vector<int>> send_dst_list(100,w);
// DATA_PER_PAC send commands merge as one
//vector<int> a(100,1);
//vector<vector<int>> send_merge(100,a);

bool
GarnetSyntheticTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);  // packet is injected fanxi
    return true;
}

void
GarnetSyntheticTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
GarnetSyntheticTraffic::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

GarnetSyntheticTraffic::GarnetSyntheticTraffic(const Params *p)
    : MemObject(p),
      tickEvent([this]{ tick(); }, "GarnetSyntheticTraffic tick",
                false, Event::CPU_Tick_Pri),
      cachePort("GarnetSyntheticTraffic", this),
      retryPkt(NULL),
      size(p->memory_size),
      blockSizeBits(p->block_offset),
      numDestinations(p->num_dest),
      simCycles(p->sim_cycles),
      numPacketsMax(p->num_packets_max),
      numPacketsSent(0),
      singleSender(p->single_sender),
      singleDest(p->single_dest),
      trafficType(p->traffic_type),
      injRate(p->inj_rate),
      injVnet(p->inj_vnet),
      if_routerless(p->if_routerless),
      precision(p->precision),
      responseLimit(p->response_limit),
      masterId(p->system->getMasterId(name()))
{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    initTrafficType();
    if (trafficStringToEnum.count(trafficType) == 0) {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }
    traffic = trafficStringToEnum[trafficType];

    id = TESTER_NETWORK++;
    DPRINTF(GarnetSyntheticTraffic,"Config Created: Name = %s , and id = %d\n",
            name(), id);

    // std::cout << "fanxi added when new GarnetSyntheticTraffic, name()= "<< name() <<" id = " << id << std::endl;
    // fanxi added when new GarnetSyntheticTraffic, name()= system.cpu06 id = 6
}

BaseMasterPort &
GarnetSyntheticTraffic::getMasterPort(const std::string &if_name, PortID idx)
{
    // called by 16 times if_name="test"
    //std::cout << "fanxi added when getMasterPort, if_name= " << if_name <<" idx= " <<idx <<std::endl;
    if (if_name == "test")
        return cachePort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

void init_recv_packet_files(int id){
    std::string file;
    file = recv_pac_file+std::to_string(id)+".txt";
	ofstream OutFile(file);
	OutFile << std::to_string(0); 
    OutFile.close();    
}

void init_send_command_output(int id){
    std::string file;
    file = "./../output_info/send_command_info/"+std::to_string(id)+".txt";
	ofstream OutFile(file);
    OutFile.close();    
}

void init_send_data(int id){
    std::string file;
    file = node2ni+std::to_string(id)+".txt";
	ofstream OutFile(file);
    OutFile.close();    
}

void init_cpu_output(int id){
    std::string file;
    std::string file1;
    file = Traffic_output_file+std::to_string(id)+".txt";
	ofstream OutFile(file);
    OutFile.close();    
}

// wxy add in 4.14
// 控制第一层的节点统一开始
void set_start(int flag){
    std::string file;
	file = Traffic_output_file+"start_signal.txt";
	ofstream OutFile(file);
	OutFile << std::to_string(flag); 
	OutFile.close();
}

bool check_start_signal(){
    std::string file;
    file = Traffic_output_file+"start_signal.txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    string s;
    while(getline(infile,s))
    {
        //std::cout<<"fanxi added, recv_packets ing, id= " << id <<" packets="<<s<<std::endl;
    }
    infile.close();             //关闭文件输入流 
    return atoi(s.c_str());
}

void
GarnetSyntheticTraffic::init_task_file(){
    std::string file;
	file = Task_folder+std::to_string(id)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    int read_line_num=0;
    std::string task_line;
    while(getline(infile,task_line))
    {
        task_file.push_back(task_line);
        read_line_num += 1;
    }
    //std::cout<<"wxy added in task_file : cpu_id"<<id<<" task_line_num:"<<read_line_num<<std::endl;
    infile.close();             //关闭文件输入流 

    if (read_line_num == 0){ //文件最后会有个空行
        working_cpu_flag = 0;
    }
    else {
        //std::cout<<"fanxi added, get_task, id= " << id <<" linenum=" <<line_num << " task ="<< current_task_line <<std::endl;
        working_cpu_flag = 1;  
    }
    std::cout<<"wxy added in task_file : cpu_id"<<id<<" cpu_woring_flag:"<<working_cpu_flag<<" task_line_num:"<<read_line_num<<std::endl;
}

void
GarnetSyntheticTraffic::init()
{   
    // std::cout << "fanxi added when GarnetSyntheticTraffic::init()"  <<std::endl;
    // called by 16 times
    cpu_status = IDLE;
    numPacketsSent = 0;
    current_line_num = 0;
    pic_num = 0;
    init_cpu_output(id);
    init_send_data(id);
    send_dst_list.resize(100,0);
    init_task_file();
    total_packet_recv_previous = 0;
    packets_to_send = 0;

    start_node_flag = 0;
    for(int i = 0; i < start_node_num; i++){
        if( id == start_node[i]){
            start_node_flag = 1;
            break;
        } 
    }
    start_signal = 0;
    if(id == node0){
        set_start(0);
    }
    wait_cycle = 0;

    std::string file;
    file = "./send_info.txt";
	ofstream OutFile(file); 
    OutFile.close();
}

void
GarnetSyntheticTraffic::completeRequest(PacketPtr pkt)
{
    //std::cout << "fanxi added when GarnetSyntheticTraffic::completeRequest()"  <<std::endl;

    DPRINTF(GarnetSyntheticTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            pkt->req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
}

int recv_packets(int id ,int pic_num)
{
	std::string file;
    file = recv_pac_file+std::to_string(id)+"_"+std::to_string(pic_num)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    string s;
    while(getline(infile,s))
    {
        //std::cout<<"fanxi added, recv_packets ing, id= " << id <<" packets="<<s<<std::endl;
    }
    infile.close();             //关闭文件输入流 
    return atoi(s.c_str());
}

// 1代表读到task 0 代表没有读到，文件结束
int GarnetSyntheticTraffic::get_task(int id,int line_num)
{
	std::string file;
	file = Task_folder+std::to_string(id)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    int read_line_num=0;
    while(getline(infile,current_task_line))
    {
        if (read_line_num == line_num){
            break;
        }
        read_line_num += 1;
    }
    infile.close();             //关闭文件输入流 
    if (read_line_num < line_num || current_task_line == ""){ //文件最后会有个空行
        return 0;
    }
    else {
        //std::cout<<"fanxi added, get_task, id= " << id <<" linenum=" <<line_num << " task ="<< current_task_line <<std::endl;
        return 1;   
    }
    
}

vector<string> split(const string &str, const string &pattern)
{
    vector<string> res;
    if(str == "")
        return res;
    //在字符串末尾也加入分隔符，方便截取最后一段
    string strs = str + pattern;
    size_t pos = strs.find(pattern);

    while(pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        //去掉已分割的字符串,在剩下的字符串中进行分割
        strs = strs.substr(pos+1, strs.size());
        pos = strs.find(pattern);
    }

    return res;
}

void tell_mem_send_data(std::string src_mem_index,std::string num_wait_packets,int id) 
{
	std::string file;
    file = "./../cpu_task/"+src_mem_index+".txt";
    fstream f;

    std::string message_to_write = "send ";
    message_to_write.append(std::to_string(id));
    message_to_write.append(" ");
    message_to_write.append(num_wait_packets);
    //追加写入,在原来基础上加了ios::app 
	f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
	 
}

//flag == 1 : record communicaion time ; info = communication time;
//== 0 : finish  ; info = current_line_num; 
void output_data(int id, int info, bool flag)
{
    std::string file;
    file = "./../cpu_output/"+std::to_string(id)+".txt";
    fstream f;
    std::string message_to_write;
    if ( flag )
    {
        message_to_write.append("communication time = ");
        message_to_write.append(std::to_string(info));
        message_to_write.append(" ;  curTick = ");
        message_to_write.append(std::to_string(curTick()));
    }
    else
    {
        message_to_write.append("finish : curTick = ");
        message_to_write.append(std::to_string(curTick()));
        message_to_write.append("  ;  curTaskLineNum = ");
        message_to_write.append(std::to_string(info));
    }
    //追加写入,在原来基础上加了ios::app 
	f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
}

//flag == 1 : record task read time ; info = task_line_num;
//== 0 : record send/finish successfully ; info = dst_node; 
//now use
void output_data_1(int id, int info, std::string type, bool flag, int pic_num)
{
    std::string file;
    //file = Traffic_output_file+std::to_string(id)+"_"+std::to_string(pic_num)+".txt";
    file = Traffic_output_file+std::to_string(id)+".txt";
    fstream f;
    std::string message_to_write;

    if(flag == 1){
        message_to_write.append(type);
        message_to_write.append("_at_time ");
        message_to_write.append(std::to_string(curTick()));
        message_to_write.append(" ;start_task_line_num ");
        message_to_write.append(std::to_string(info));
    }
    else{
        if(type == "send"){
            message_to_write.append("Send_successfully_to_node ");
            message_to_write.append(std::to_string(info));
        }
        else if(type == "finish"){
            message_to_write.append("Finish_successfully_at_picture ");
            message_to_write.append(std::to_string(info));
        }
        message_to_write.append(" at_time ");
        message_to_write.append(std::to_string(curTick()));
    }
    //追加写入,在原来基础上加了ios::app 
    f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
}

void
GarnetSyntheticTraffic::tick_pre_0()
{   
    std::cout << "cpu id"<<id<<" status:" << cpu_status <<" current_line_num:"<< current_line_num << std::endl;
    std::cout << "cpu id"<<id<<" cpu_work_stats:" << cpu_work_stats <<std::endl;
    
    int if_get_task;
    bool sendAllowedThisCycle = false;
    // idle status : read task file
    if (cpu_status == IDLE){
        if_get_task = get_task(id, current_line_num);
        
        
        if (if_get_task == 0){
            cpu_status = IDLE;
        }
        
        else{// 解析task_line
            current_line_num += 1;
            vector<string>  current_task;
            current_task  = split(current_task_line," ");

            if (current_task[0] == "wait"){
                std::cout << "== wait" << std::endl;
                std::cout << "== ID" << id <<"  wait curTick : " << curTick() <<std::endl;
                tick_pre = curTick();

                cpu_status = WORKIING;
                cpu_work_stats = WORK_WAIT;
                num_packet_wait = atoi(current_task[1].c_str());
                std::string str_num_wait_packets = current_task[1];
                std::string str_src_mem_index = current_task[2];
                tell_mem_send_data(str_src_mem_index,  str_num_wait_packets,  id);
            }
            else if (current_task[0] == "cal"){
                std::cout << "== cal" << std::endl;
                std::cout << "== cal curTick : " << curTick() <<std::endl;

                communication_tick = curTick() - tick_pre;
                std::cout << "== ID"<<id << "  comm_tick : "<< communication_tick << std::endl;
                
                output_data(id, communication_tick , 1);

                cpu_status = WORKIING;
                cpu_work_stats = WORK_CAL;

                stringstream stream;            //声明一个stringstream变量
                stream << current_task[1];      //向stream中插入字符串"1234"
                stream >> cal_cycles;           // 初始化cal_cycles 
                cycles_caled = 0;
            }
            else if (current_task[0] == "send"){
                std::cout << "== send" << std::endl;
                cpu_status = WORKIING;
                cpu_work_stats = WORK_SEND; 
                packets_to_send = atoi(current_task[2].c_str());
                send_dst = atoi(current_task[1].c_str());
                packets_sent = 0;
            }
            else if (strstr(current_task[0].c_str(), "finish") != NULL ) { // 最后一行current_task[0]会多一个终结符
                cpu_status = FINISH;
                cpu_work_stats = WORK_IDLE;

                output_data(id, current_line_num , 0);
            }
        }
    }

    // working status
    else if (cpu_status == WORKIING){
        if (cpu_work_stats == WORK_WAIT){
            int packet_recv = recv_packets(id , pic_num) - total_packet_recv_previous;
            if (packet_recv == num_packet_wait){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
                total_packet_recv_previous += packet_recv;
            }
            // 否则维持wait状态
        }
        else if (cpu_work_stats == WORK_SEND){
            if (packets_sent == packets_to_send){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                sendAllowedThisCycle = true;
            }
        }

        else if (cpu_work_stats == WORK_CAL){
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                cycles_caled += 1;
            }
        }
    }
    

    
	
	// std::cout<<" fanxi added GarnetSyntheticTraffic::tick(), id= "<< id << std::endl;
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    
    // double injRange = pow((double) 10, (double) precision);
    // unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    // if (trySending < injRate*injRange)
    //     sendAllowedThisCycle = true;
    // else
    //     sendAllowedThisCycle = false;

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
		// std::cout<<" fanxi added GarnetSyntheticTraffic: sendAllowedThisCycle id = "<< id << std::endl; 
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable){
            generatePkt(send_dst);
            packets_sent += 1;
        }
           
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
    {
        if(cpu_status != FINISH) output_data(id, current_line_num, 0);
        exitSimLoop("Network Tester completed simCycles");
    }
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

//wxy add in 3.30
//输出有关于合并send指令后的send指令执行情况，输出到send_command_info文件夹
void send_command_output(int id, std::string line, int pic_num){
    ofstream OutFile;
    std::string file;
	file = "./../output_info/send_command_info/"+std::to_string(id)+"_"+std::to_string(pic_num)+".txt";
    OutFile.open(file, ios::app);
	OutFile <<line <<std::endl; 
	OutFile.close(); 
}

void pic_recv_all(int id, int flag){
    std::string file;
	file = "./../recv/layer2_recv.txt";
	ofstream OutFile(file);
	OutFile << std::to_string(flag); 
    //std::cout<<"fanxi added, update_recv_packets ing, id= " << id <<" packets="<<num_recv_packet<<std::endl;
	OutFile.close();
}

bool check_2_all(){
    std::string file;
    file = "./../recv/layer2_recv.txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    string s;
    while(getline(infile,s))
    {
        //std::cout<<"fanxi added, recv_packets ing, id= " << id <<" packets="<<s<<std::endl;
    }
    infile.close();             //关闭文件输入流 
    return atoi(s.c_str());
}

//wxy add in 4.1
//修改finish，使得他从头开始读任务
void
GarnetSyntheticTraffic::tick()
{  
    
    int if_get_task = 0;
    bool sendAllowedThisCycle = false;
    std::string send_output_line;
    // idle status : read task file

    if (cpu_status == IDLE){
        if(start_signal || !start_node_flag){
            //if_get_task = get_task(id, current_line_num);
            if(working_cpu_flag){
                current_task_line = task_file[current_line_num];
                if_get_task = working_cpu_flag;
            }
            if (id== node0 && current_line_num == 3){
                set_start(0);
            }
        }
        else{
            start_signal = check_start_signal();
        }

        if (id == node0 && !start_signal){
            wait_cycle += 1;
            if (wait_cycle >= start_cycle){
                set_start(1);
            }
        }
            
        if (if_get_task == 0){
            cpu_status = IDLE;
        }
            
        else{// 解析task_line
            current_line_num += 1;
            vector<string>  current_task;
            current_task  = split(current_task_line," ");
            if (pic_num < 100)
                output_data_1(id, current_line_num-1, current_task[0], 1, pic_num);

            if (current_task[0] == "wait"){
                //std::cout << "== wait" << std::endl;
                //std::cout << "== ID" << id <<"  wait curTick : " << curTick() <<std::endl;
                tick_pre = curTick();

                cpu_status = WORKIING;
                cpu_work_stats = WORK_WAIT;
                num_packet_wait = atoi(current_task[1].c_str());
                std::string str_num_wait_packets = current_task[1];
                //std::string str_src_mem_index = current_task[2];
                //tell_mem_send_data(str_src_mem_index,  str_num_wait_packets,  id);
            }
            else if (current_task[0] == "cal"){
                //std::cout << "== cal" << std::endl;
                //std::cout << "== cal curTick : " << curTick() <<std::endl;

                communication_tick = curTick() - tick_pre;
                //std::cout << "== ID"<<id << "  comm_tick : "<< communication_tick << std::endl;
                    
                //output_data(id, communication_tick , 1);

                cpu_status = WORKIING;
                cpu_work_stats = WORK_CAL;

                stringstream stream;            //声明一个stringstream变量
                stream << current_task[1];      //向stream中插入字符串"1234"
                stream >> cal_cycles;           // 初始化cal_cycles 
                cycles_caled = 0;
            }
            else if (current_task[0] == "send"){
                //std::cout << "== send" << std::endl;
                cpu_status = WORKIING;
                cpu_work_stats = WORK_SEND; 
                //packets_to_send = atoi(current_task[2].c_str());
                //packets_to_send = current_task.size() - 1;
                send_dst = atoi(current_task[1].c_str());
                flit_num = atoi(current_task[2].c_str());

                packets_to_send = 1;
                    
                //send_output_line = "current packets_to_send = "+ std::to_string(packets_to_send) + "     ; curTick = " + std::to_string(curTick());
                //send_command_output(id, send_output_line);
                packets_sent = 0;
            }
            else if (strstr(current_task[0].c_str(), "finish") != NULL ) { // 最后一行current_task[0]会多一个终结符

                cpu_status = FINISH;
                cpu_work_stats = WORK_IDLE;
                
            }
        }
        if (cpu_status == FINISH){
            /*
            if(id == 0 || id == 6 || id == 12 || id == 18 || id == 24 || id == 30 || id == 1 || id == 7 || id == 19 || id == 25){
                if(check_2_all() && pic_num < 300){
                    cpu_status = IDLE;
                    cpu_work_stats = WORK_IDLE;
                    current_line_num = 0;
                    numPacketsSent = 0;
                    if(pic_num < 100)
                        output_data_1(id, pic_num, "finish", 0, pic_num);
                    pic_num += 1;
                }
            }
            */
            //else{
                cpu_status = IDLE;
                cpu_work_stats = WORK_IDLE;
                current_line_num = 0;
                numPacketsSent = 0;
                start_signal = 0;
                wait_cycle = 0;
                //total_packet_recv_previous = 0;
                if(pic_num <= FINISH_PIC)
                    output_data_1(id, pic_num, "finish", 0, pic_num);
                pic_num += 1;
                if(pic_num > FINISH_PIC){
                    cpu_status = FINISH;
                    cpu_work_stats = WORK_IDLE;
                }
            //}
        }
    }
    // working status
    if (cpu_status == WORKIING){
        if (cpu_work_stats == WORK_WAIT){
            int packet_recv = recv_packets(id,pic_num);
            //pic_recv_all(id,0);v
            if (packet_recv >= num_packet_wait){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
                total_packet_recv_previous += num_packet_wait;
                //if(id == 13)
                    //pic_recv_all(id,1);
            }
            // 否则维持wait状态
        }
        else if (cpu_work_stats == WORK_SEND){
            /*
            if (packets_sent == packets_to_send){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                send_dst = send_dst_list[packets_sent];
                sendAllowedThisCycle = true;
            }
            */
            //send_output_line = "!!! dst_node " + std::to_string(send_dst) + "  ;  num = " + std::to_string(send_merge[send_dst]) + "     ; curTick = " + std::to_string(curTick());
            //send_command_output(id, send_output_line, pic_num);
            sendAllowedThisCycle = true;
            if(pic_num < 100)
                output_data_1(id, send_dst, "send", 0, pic_num);
            if (packets_sent == packets_to_send - 1){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
                packets_to_send = 0;
            }
        }

        else if (cpu_work_stats == WORK_CAL){
            /*
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                cycles_caled += 1;
            }
            */
            cycles_caled += 1;
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
        }
    }

    std::cout << "cpu id"<<id<<" status:" << cpu_status <<" current_line_num:"<< current_line_num << std::endl;
    std::cout << "cpu id"<<id<<" cpu_work_stats:" << cpu_work_stats <<" cur_tick:"<<curTick()<<std::endl;
	
	// std::cout<<" fanxi added GarnetSyntheticTraffic::tick(), id= "<< id << std::endl;
    // wxy add in 4.9
    //if (++noResponseCycles >= responseLimit) {
    //    fatal("%s deadlocked at cycle %d\n", name(), curTick());
    //}

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    
    // double injRange = pow((double) 10, (double) precision);
    // unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    // if (trySending < injRate*injRange)
    //     sendAllowedThisCycle = true;
    // else
    //     sendAllowedThisCycle = false;

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
		// std::cout<<" fanxi added GarnetSyntheticTraffic: sendAllowedThisCycle id = "<< id << std::endl; 
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable){
            generatePkt(send_dst);
            packets_sent += 1;
        }
           
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
    {
        if(cpu_status != FINISH) output_data(id, current_line_num, 0);
        exitSimLoop("Network Tester completed simCycles");
    }
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

//wxy add in 4.1
//实现与NI的数据交互，输入此刻传输的信息，输出文件夹send_data
void send_packet_data(int data, int data2, int src)
{
	std::string file;
    file = node2ni+std::to_string(src)+".txt";
	ofstream OutFile;
    OutFile.open(file,ios::app);
    OutFile<< std::to_string(data)<<" "<<std::to_string(data2)<<std::endl;;  
    OutFile.close(); 
}

void
GarnetSyntheticTraffic::generatePkt(int send_dst)
{
	//std::cout<<" fanxi added GarnetSyntheticTraffic: generatePkt(), id: " <<id << std::endl; 
    int num_destinations = numDestinations;
    int radix = (int) sqrt(num_destinations);
    unsigned destination = id;
    int dest_x = -1;
    int dest_y = -1;
    int source = id;
    int src_x = id%radix;
    int src_y = id/radix;

    if (singleDest >= 0)
    {
        destination = singleDest;
    } else if (traffic == UNIFORM_RANDOM_) {
        destination = random_mt.random<unsigned>(0, num_destinations - 1);
    } else if (traffic == BIT_COMPLEMENT_) {
        dest_x = radix - src_x - 1;
        dest_y = radix - src_y - 1;
        destination = dest_y*radix + dest_x;
    } else if (traffic == BIT_REVERSE_) {
        unsigned int straight = source;
        unsigned int reverse = source & 1; // LSB

        int num_bits = (int) log2(num_destinations);

        for (int i = 1; i < num_bits; i++)
        {
            reverse <<= 1;
            straight >>= 1;
            reverse |= (straight & 1); // LSB
        }
        destination = reverse;
    } else if (traffic == BIT_ROTATION_) {
        if (source%2 == 0)
            destination = source/2;
        else // (source%2 == 1)
            destination = ((source/2) + (num_destinations/2));
    } else if (traffic == NEIGHBOR_) {
            dest_x = (src_x + 1) % radix;
            dest_y = src_y;
            destination = dest_y*radix + dest_x;
    } else if (traffic == SHUFFLE_) {
        if (source < num_destinations/2)
            destination = source*2;
        else
            destination = (source*2 - num_destinations + 1);
    } else if (traffic == TRANSPOSE_) {
            dest_x = src_y;
            dest_y = src_x;
            destination = dest_y*radix + dest_x;
    } else if (traffic == TORNADO_) {
        dest_x = (src_x + (int) ceil(radix/2)) % radix;
        dest_y = src_y;
        destination = dest_y*radix + dest_x;
    }
    else {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  send_dst;  // fanxi modified
    paddr <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // GarnetSyntheticTraffic assumes the Garnet_standalone coherence protocol
    // which models three message classes/virtual networks.
    // These are: request, forward, response.
    // requests and forwards are "control" packets (typically 8 bytes),
    // while responses are "data" packets (typically 72 bytes).
    //
    // Life of a packet from the tester into the network:
    // (1) This function generatePkt() generates packets of one of the
    //     following 3 types (randomly) : ReadReq, INST_FETCH, WriteReq
    // (2) mem/ruby/system/RubyPort.cc converts these to RubyRequestType_LD,
    //     RubyRequestType_IFETCH, RubyRequestType_ST respectively
    // (3) mem/ruby/system/Sequencer.cc sends these to the cache controllers
    //     in the coherence protocol.
    // (4) Network_test-cache.sm tags RubyRequestType:LD,
    //     RubyRequestType:IFETCH and RubyRequestType:ST as
    //     Request, Forward, and Response events respectively;
    //     and injects them into virtual networks 0, 1 and 2 respectively.
    //     It immediately calls back the sequencer.
    // (5) The packet traverses the network (simple/garnet) and reaches its
    //     destination (Directory), and network stats are updated.
    // (6) Network_test-dir.sm simply drops the packet.
    //
    MemCmd::Command requestType;

    RequestPtr req = nullptr;
    Request::Flags flags;

    // Inject in specific Vnet
    // Vnet 0 and 1 are for control packets (1-flit)
    // Vnet 2 is for data packets (5-flit)
    int injReqType = injVnet;
    //std::cout<<" fanx added the generated packet: destination= "<< send_dst << "; injVnet=" <<injVnet << std::endl;

    // 依据节点数目+src+dst确定注入的Vnet id
    // 如果是routerless设置，执行以下函数决定vnet；否则维持原始设置
    if (if_routerless == 1) {
        if (num_destinations==4) // 2*2 mesh
            injReqType = vnet_table_2_2[source][send_dst] + 2;
        else if (num_destinations==16) //4*4 mesh
            injReqType = vnet_table_4_4[source][send_dst] + 2;
        else if (num_destinations==36) //6*6 mesh
            injReqType = vnet_table_6_6[source][send_dst] + 2;
        else if (num_destinations==64) //8*8 mesh
            injReqType = vnet_table_8_8[source][send_dst] + 2;
        else {
            std::cout << "routerless mesh size is not supported now" << std::endl;
            assert(0);} 
        std::cout << "tempadded the injReqType=" << injReqType << " source=" <<source<<" destination=" <<send_dst
            << "num_destinations" <<num_destinations << std::endl;
    }

    if (injReqType==-1) // -1 代表random(0,2),其他代表自己本身
    {
        // randomly inject in any vnet
        injReqType = random_mt.random(0, 2);
    }

    if (injReqType == 0) {
        // generate packet for virtual network 0
        requestType = MemCmd::ReadReq;
        req = new Request(paddr, access_size, flags, masterId);
    } else if (injReqType == 1) {
        // generate packet for virtual network 1
        requestType = MemCmd::ReadReq;
        flags.set(Request::INST_FETCH);
        req = new Request(
            0, 0x0, access_size, flags, masterId, 0x0, 0);
        req->setPaddr(paddr);
    } 
    else if (injReqType == 2) {
        // generate packet for virtual network 2
        requestType = MemCmd::WriteReq;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else  if (injReqType == 3) {
        requestType = MemCmd::WriteReq1;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else  if (injReqType == 4) {
        requestType = MemCmd::WriteReq2;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 5) {
        requestType = MemCmd::WriteReq3;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 6) {
        requestType = MemCmd::WriteReq4;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 7) {
        requestType = MemCmd::WriteReq5;
        req = new Request(paddr, access_size, flags, masterId);
    }

    else if (injReqType == 8) {
        requestType = MemCmd::WriteReq6;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 9) {
        requestType = MemCmd::WriteReq7;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 10) {
        requestType = MemCmd::WriteReq8;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 11) {
        requestType = MemCmd::WriteReq9;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 12) {
        requestType = MemCmd::WriteReq10;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 13) {
        requestType = MemCmd::WriteReq11;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 14) {
        requestType = MemCmd::WriteReq12;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 15) {
        requestType = MemCmd::WriteReq13;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 16) {
        requestType = MemCmd::WriteReq14;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 17) {
        requestType = MemCmd::WriteReq15;
        req = new Request(paddr, access_size, flags, masterId);
    }
    else if (injReqType == 18) {
        requestType = MemCmd::WriteReq16;
        req = new Request(paddr, access_size, flags, masterId);
    }
    //std::cout<<"wxy added the generated packet: masterId =  "<< masterId << std::endl;
    req->setContext(id);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(GarnetSyntheticTraffic,
            "Generated packet with destination %d, embedded in address %x\n",
            destination, req->getPaddr());

    PacketPtr pkt = new Packet(req, requestType);
    pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;

    send_packet_data(pic_num, flit_num,id);
    sendPkt(pkt);
}

void
GarnetSyntheticTraffic::initTrafficType()
{
    trafficStringToEnum["bit_complement"] = BIT_COMPLEMENT_;
    trafficStringToEnum["bit_reverse"] = BIT_REVERSE_;
    trafficStringToEnum["bit_rotation"] = BIT_ROTATION_;
    trafficStringToEnum["neighbor"] = NEIGHBOR_;
    trafficStringToEnum["shuffle"] = SHUFFLE_;
    trafficStringToEnum["tornado"] = TORNADO_;
    trafficStringToEnum["transpose"] = TRANSPOSE_;
    trafficStringToEnum["uniform_random"] = UNIFORM_RANDOM_;
}

void
GarnetSyntheticTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
GarnetSyntheticTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

GarnetSyntheticTraffic *
GarnetSyntheticTrafficParams::create()
{   
    //std::cout<<" fanxi added GarnetSyntheticTrafficParams::create() " << std::endl;  
    // called by 16 times at the begining to new 16 cpus
    return new GarnetSyntheticTraffic(this);
}


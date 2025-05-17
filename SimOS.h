// Bode Chiu 340 Project Shostak
// SimOS.h
#ifndef SIMOS_H
#define SIMOS_H

#include <vector>
#include <queue>
#include <string>
#include <iostream>
#include <map>
#include <algorithm>

struct FileReadRequest {
    int PID{0};
    std::string fileName{""};
};

struct MemoryItem {
    unsigned long long itemAddress;
    unsigned long long itemSize;
    int PID;
};

using MemoryUse = std::vector<MemoryItem>;

constexpr int NO_PROCESS{-1};

struct PCB {
    int PID;
    int priority;
    unsigned long long size;
    int parentPID = NO_PROCESS;
    bool isZombie = false;
    bool isWaiting = false;
    std::vector<int> children;
};


class SimOS {
public:
    SimOS(int numberOfDisks, unsigned long long amountOfRAM, unsigned long long sizeOfOS);

    bool NewProcess(unsigned long long size, int priority);
    bool findWorstFit(unsigned long long size, unsigned long long& addressOut);
    void mergeMemoryHoles();
    bool SimFork();
    void SimExit();
    void removeProcessEverywhere(int pid);
    void SimWait();
    void DiskReadRequest(int diskNumber, std::string fileName);
    void DiskJobCompleted(int diskNumber);

    int GetCPU();
    std::vector<int> GetReadyQueue();
    MemoryUse GetMemory();
    void sortMemoryByAddress(MemoryUse& memList);
    FileReadRequest GetDisk(int diskNumber);
    std::queue<FileReadRequest> GetDiskQueue(int diskNumber);

private:
    int cpuPID = NO_PROCESS; 
    unsigned long long totalRAM;
    MemoryUse memory;
    std::vector<MemoryItem> memoryHoles; // Tracks free memory holes
    int numDisks;
    std::vector<std::queue<FileReadRequest>> diskQueues; //all diskQueues
    std::vector<FileReadRequest> disks; //current process on disc
    std::map<int, PCB> processes; // Maps PID to PCB struct
    int nextPID;

    
    struct ComparePriority {
        bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) const {
            return a.second < b.second; // higher priority value means higher priority
        }
    };
    //Storing PID, Priortity
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, ComparePriority> readyQueue;

};

#endif
// Bode Chiu 340 Project Shostak
// SimOS.cpp
#include "SimOS.h"

/**
@param: numberOfDisks - the number of hard disks in the simulated system
@param: amountOfRAM - the total amount of RAM available in the simulated system
@param: sizeOfOS - the memory size required by the OS process (PID 1), which starts at memory address 0
@post: Initializes the SimOS object, sets up memory, creates the OS process, and initializes disk queues and tracking structures
*/

SimOS::SimOS(int numberOfDisks, unsigned long long amountOfRAM, unsigned long long sizeOfOS)
    : numDisks(numberOfDisks), totalRAM(amountOfRAM), nextPID(2) {
    
    // 1. Initialize disk structures
    diskQueues.resize(numDisks);  // FIFO queues for each disk
    disks.resize(numDisks);
    
    // 2. Create the OS process (PID 1)
    MemoryItem osMemory;
    osMemory.PID = 1;
    osMemory.itemAddress = 0;
    osMemory.itemSize = sizeOfOS;
    
    memory.push_back(osMemory); // OS in memory
    cpuPID = 1;                 // OS starts on CPU

    // 3. Create the first memory hole (after the OS)
    if (sizeOfOS < totalRAM) {
        MemoryItem hole;
        hole.itemAddress = sizeOfOS;
        hole.itemSize = totalRAM - sizeOfOS;
        hole.PID = NO_PROCESS;  // Empty PID because this is the memory hole
        memoryHoles.push_back(hole);
    }
}

/**
@param: size - the amount of memory the new process will require
@param: priority - the scheduling priority of the new process (higher number = higher priority)
@return: return true if the process is successfully created and memory is allocated; false if memory is insufficient
@post: Adds the process to the ready queue or CPU, assigns it a unique PID, and allocates memory using worst-fit
*/

bool SimOS::NewProcess(unsigned long long size, int priority) {
    unsigned long long addr = 0;
    if (!findWorstFit(size, addr)) {
        return false; // No space
    }

    // Create memory block
    MemoryItem memItem{addr, size, nextPID};
    memory.push_back(memItem);

    // Create PCB
    PCB pcb;
    pcb.PID = nextPID;
    pcb.priority = priority;
    pcb.size = size;
    processes[nextPID] = pcb;

    // Schedule process
    if (cpuPID == 1 || processes[cpuPID].priority < priority || cpuPID == NO_PROCESS) {
        if (cpuPID != NO_PROCESS) {
            readyQueue.push({cpuPID, processes[cpuPID].priority});
        }
        cpuPID = nextPID;
    } else {
        readyQueue.push({nextPID, priority});
    }

    this->nextPID ++;
    return true;
}

/**
@param: size - the memory size requested by the process
@param: addressOut - reference to store the memory address where the block will be allocated
@return: true if a suitable memory hole was found and updated; false if insufficient space
@post: Finds the worst-fitting memory hole and updates it to reflect the allocation
*/

bool SimOS::findWorstFit(unsigned long long size, unsigned long long& addressOut) {
    mergeMemoryHoles();
    unsigned long long largestSize = 0;
    int index = -1;

    for (int i = 0; i < memoryHoles.size(); ++i) {
        if (memoryHoles[i].itemSize >= size && memoryHoles[i].itemSize > largestSize) {
            largestSize = memoryHoles[i].itemSize;
            index = i;
        }
    }

    if (index == -1) return false;

    
    addressOut = memoryHoles[index].itemAddress;
    
    // Update hole to reflect new process allocation
    if (memoryHoles[index].itemSize == size) {
        memoryHoles.erase(memoryHoles.begin() + index);
    } else {
        memoryHoles[index].itemAddress += size;
        memoryHoles[index].itemSize -= size;
    }
    
    return true;
}

/**
@post: Merges adjacent memory holes into single larger holes
       to maintain a clean and accurate representation of free memory
*/

void SimOS::mergeMemoryHoles() {
    sortMemoryByAddress(memoryHoles);
    
    std::vector<MemoryItem> merged;
    for (int i = 0; i < memoryHoles.size(); ++i) {
        if (merged.empty() || merged.back().itemAddress + merged.back().itemSize == memoryHoles[i].itemAddress) {
            merged.push_back(memoryHoles[i]);
        } else {
            // Merge with previous
            unsigned long long newEnd = std::max(
                merged.back().itemAddress + merged.back().itemSize,
                memoryHoles[i].itemAddress + memoryHoles[i].itemSize
            );
            merged.back().itemSize = newEnd - merged.back().itemAddress;
        }
    }

    memoryHoles = merged;
}

/**
@param: memList - the vector of memory items to be sorted
@post: Sorts memory items in increasing order by their memory address
*/

void SimOS::sortMemoryByAddress(MemoryUse& memList) {
    std::sort(memList.begin(), memList.end(), [](const MemoryItem& a, const MemoryItem& b) {
        return a.itemAddress < b.itemAddress;
    });
}

/**
@return: true if the forked child process is created successfully; false if memory is insufficient or the OS is running
@post: Allocates memory using worst-fit, clones parent's priority/size, creates child PCB, and adds to readyQueue
*/

bool SimOS::SimFork() {
    // Ignore if the OS is running
    if (cpuPID == 1) return false;

    // Get parent PCB
    PCB parent = processes[cpuPID];
    unsigned long long addr;

    // Try to allocate memory using worst-fit
    if (!findWorstFit(parent.size, addr)) {
        return false; // Not enough memory
    }

    // Create memory block for child process
    MemoryItem memItem;
    memItem.PID = nextPID;
    memItem.itemAddress = addr;
    memItem.itemSize = parent.size;
    memory.push_back(memItem);

    // Create child PCB
    PCB child;
    child.PID = nextPID;
    child.priority = parent.priority;
    child.size = parent.size;
    child.parentPID = cpuPID;
    processes[nextPID] = child;

    // Update parent's children list
    processes[cpuPID].children.push_back(nextPID);

    // Add child to ready queue
    readyQueue.push({nextPID, child.priority});

    ++nextPID;
    return true;
}

/**
@post: Terminates the currently running process (except for the OS). 
       If the parent is waiting (has called SimWait), the parent resumes execution immediately 
       and the process is fully removed. If not, the process becomes a zombie.
       Cascading termination is applied to all descendants of the exiting process.
*/

void SimOS::SimExit() {
    if (cpuPID == 1) return;  // OS never exits

    int exitingPID = cpuPID;
    cpuPID = NO_PROCESS;

    // Recursively terminate children
    for (int childPID : processes[exitingPID].children) {
        cascadingTerminate(childPID);
    }

    int parentID = processes[exitingPID].parentPID;

    if (parentID != NO_PROCESS && processes.find(parentID) != processes.end()) {
        PCB& parent = processes[parentID];

        if (parent.isWaiting) {
            // Parent is waiting — clean up immediately and resume it
            parent.isWaiting = false;

            if (cpuPID == NO_PROCESS || cpuPID == 1 || processes[cpuPID].priority < parent.priority) {
                if (cpuPID != NO_PROCESS) {
                    readyQueue.push({cpuPID, processes[cpuPID].priority});
                }
                cpuPID = parentID;
            } else {
                readyQueue.push({parentID, parent.priority});
            }

            // Remove exiting process fully
            removeProcessEverywhere(exitingPID);
        } else {
            // Parent has not called wait yet so turn into zombie
            processes[exitingPID].isZombie = true;
        }
    } else {
        // No parent so just clean up from everywhere
        removeProcessEverywhere(exitingPID);
    }
}

/**
@param: pid - the process ID to be terminated along with its descendants
@post: Recursively terminates the specified process and all of its descendants. 
       Each terminated process is removed from memory, queues, CPU, and process table.
*/

void SimOS::cascadingTerminate(int pid) {
    if (processes.find(pid) == processes.end()) return;

    // First recursively kill children
    for (int child : processes[pid].children) {
        cascadingTerminate(child);
    }

    // Remove this process from everywhere
    removeProcessEverywhere(pid);
}

/**
@param: pid - the process ID to be removed from all system structures
@post: Removes the process from memory, CPU, disk queues, ready queue, and the process table;
       releases its memory and reassigns the CPU if needed. Updates the memoryhole vector as well
*/

void SimOS::removeProcessEverywhere(int pid) {
    // 1. Remove from memory
    for (int i = 0; i < memory.size(); ++i) {
        if (memory[i].PID == pid) {
            // Convert memory chunk to a hole
            MemoryItem hole = memory[i];
            hole.PID = NO_PROCESS;
            memoryHoles.push_back(hole);

            memory.erase(memory.begin() + i);
            break;
        }
    }

    // 2. Remove from disks
    for (int i = 0; i < disks.size(); ++i) {
        if (disks[i].PID == pid) {
            disks[i] = FileReadRequest{};  // set to idle

            // Start next job on this disk if available
            if (!diskQueues[i].empty()) {
                disks[i] = diskQueues[i].front();
                diskQueues[i].pop();
            }
        }
    }

    // 3. Remove from diskQueues
    for (auto& queue : diskQueues) {
        std::queue<FileReadRequest> newQueue;
        while (!queue.empty()) {
            FileReadRequest fr = queue.front();
            queue.pop();
            if (fr.PID != pid) {
                newQueue.push(fr);
            }
        }
        queue = newQueue;
    }

    // 4. Remove from readyQueue
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, ComparePriority> newReadyQueue;
    while (!readyQueue.empty()) {
        auto top = readyQueue.top();
        readyQueue.pop();
        if (top.first != pid) {
            newReadyQueue.push(top);
        }
    }
    readyQueue = newReadyQueue;

    // 5. Remove from CPU if currently running
    if (cpuPID == pid) {
        cpuPID = NO_PROCESS;
    }

    // 6. Remove from process map
    processes.erase(pid);

    // Resume next highest-priority process
    if (!readyQueue.empty()) {
        auto next = readyQueue.top();
        readyQueue.pop();
        cpuPID = next.first;
    }

    // 7. Merge updated memory holes
    mergeMemoryHoles();
}

/**
@post: If a zombie child exists, reaps one and resumes the parent.
       If no zombie child exists, the current process pauses and waits.
       If waiting, the process is removed from CPU and the next highest-priority process runs.
       The OS (PID 1) ignores this instruction.
*/

void SimOS::SimWait() {
    if (cpuPID == 1) return;  // OS ignores wait

    int parentPID = cpuPID;
    PCB& parent = processes[parentPID];

    // Look for a zombie child
    for (int i = 0; i < parent.children.size(); ++i) {
        int childPID = parent.children[i];
        if (processes.find(childPID) != processes.end() && processes[childPID].isZombie) {
            // Reap this zombie
            processes.erase(childPID);

            // Remove child from parent's list
            parent.children.erase(parent.children.begin() + i);

            // Parent keeps using CPU
            return;
        }
    }

    // No zombie child found — parent must wait
    parent.isWaiting = true;
    cpuPID = NO_PROCESS;

    // Resume next highest-priority process
    if (!readyQueue.empty()) {
        auto next = readyQueue.top();
        readyQueue.pop();
        cpuPID = next.first;
    }
}

/**
@param: diskNumber - the index of the disk the request is targeting
@param: fileName - the file to be read from disk
@post: Current CPU process is removed and assigned to the disk or added to the disk's I/O queue.
       If the disk is free, the request is served immediately; otherwise queued.
       If a ready process exists, it is moved to the CPU.
       The OS (PID 1) ignores this instruction.
*/

void SimOS::DiskReadRequest(int diskNumber, std::string fileName) {
    // Ignore if diskNumber is invalid
    if (diskNumber < 0 || diskNumber >= numDisks) {
        return;
    }

    // Ignore if OS process (PID 1) is running
    if (cpuPID == 1) {
        return;
    }

    // Build the file read request for the current process
    FileReadRequest request;
    request.PID = cpuPID;
    request.fileName = fileName;

    // Remove the process from CPU
    cpuPID = NO_PROCESS; //PID of '-1'

    // If the disk is idle, serve the request immediately
    if (disks[diskNumber].PID == 0 && disks[diskNumber].fileName == "") {
        disks[diskNumber] = request;
    } else {
        // Otherwise, enqueue the request
        diskQueues[diskNumber].push(request);
    }

    // Check if there's a process waiting in the ready queue
    if (!readyQueue.empty()) {
        std::pair<int, int> next = readyQueue.top();  // (PID, priority)
        readyQueue.pop();
        cpuPID = next.first;
    }
}

/**
@param: diskNumber - the disk that has completed a job
@post: The completed process returns to the CPU if it has higher priority than the current CPU process;
       otherwise it goes to the ready queue. The next I/O request in the disk queue (if any) is assigned.
*/

void SimOS::DiskJobCompleted(int diskNumber) {
    // Check if diskNumber is valid
    if (diskNumber < 0 || diskNumber >= numDisks) {
        return;
    }

    // Get the completedd request from the disk
    FileReadRequest completedRequest = disks[diskNumber];

    // If there was an active request
    if (completedRequest.PID != 0) {
        int pid = completedRequest.PID;
        int priority = processes[pid].priority;

        // Compare with current CPU process
        if (cpuPID == 1 || processes[cpuPID].priority < priority || cpuPID == NO_PROCESS) {
            //Add current Process into readyQueue as long as not NO_PROCESS, then change the respective process on the CPU
            if (cpuPID == 1) {
                readyQueue.push({cpuPID, processes[cpuPID].priority});
            }
            cpuPID = pid;  // Put to CPU if higher Priority
        } else {
            readyQueue.push({pid, priority});  // Goes back to ready queue
        }

        // Mark disk as idle by default and we will check the I/O Queue next
        disks[diskNumber] = FileReadRequest{};
    }

    // Load the next waiting request into this disk (FIFO)
    if (!diskQueues[diskNumber].empty()) {
        disks[diskNumber] = diskQueues[diskNumber].front();
        diskQueues[diskNumber].pop();
    }
}


/**
@return: returns the PID of the process currently using the CPU
*/
int SimOS::GetCPU() {
    return cpuPID;
}


/**
@return: returns a vector of process IDs in the ready queue (in any order)
@post: does not modify the original ready queue
*/
std::vector<int> SimOS::GetReadyQueue() {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, ComparePriority> tempQueue = readyQueue;
    std::vector<int> result;

    while (!tempQueue.empty()) {
        result.push_back(tempQueue.top().first);  // Extract PID
        tempQueue.pop();
    }
    std::cout << result.size() << std::endl;
    return result;
}

/**
@return: Returns a vector containing all active memory allocations, including the OS and user processes.
@post: The returned MemoryUse list reflects the current state of memory; does not include memory holes or zombie processes.
*/

MemoryUse SimOS::GetMemory() {
    return memory;
}

/**
@param: diskNumber - the index of the disk being queried  
@return: the FileReadRequest currently being served on that disk  
@post: returns a default FileReadRequest (PID 0, "") if the disk is idle or diskNumber is invalid
*/

FileReadRequest SimOS::GetDisk(int diskNumber) {
    if (diskNumber < 0 || diskNumber >= static_cast<int>(disks.size())) {
        return FileReadRequest{};  // Invalid disk
    }

    const FileReadRequest& request = disks[diskNumber];

    if (request.PID == 0 && request.fileName == "") {
        return FileReadRequest{};
    }

    return request;
}

/**
@param: diskNumber - the index of the disk being queried  
@return: the I/O queue (FIFO) of the specified disk, starting from the next process to be served  
@post: returns an empty queue if the diskNumber is invalid
*/

std::queue<FileReadRequest> SimOS::GetDiskQueue(int diskNumber) {
    if (diskNumber < 0 || diskNumber >= static_cast<int>(diskQueues.size())) {
        return std::queue<FileReadRequest>{}; // invalid disk index
    }
    return diskQueues[diskNumber]; // return the actual queue
}

std::vector<MemoryItem> SimOS::GetMemoryHoles() {
    return memoryHoles;
}
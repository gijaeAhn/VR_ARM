//
// Created by gj on 24. 3. 5.
//


#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <csignal>

#define  RMD_MOTOR_KEY              1001
#define  DYNAMIXEL_MOTOR_KEY        1002

#define  RMD_DEBUG_KEY              1003
#define  DYNAMIXEL_DEBUG_KEY        1004


#define  RMD_MOTOR_SIZE                1    
#define  DYNAMIXEL_MOTOR_SIZE          1    

#define  RMD_ANGLE_KEY              1005
#define  DYNAMIXEL_ANGLE_KEY        1006


#define  POSITION_SET_KEY           1007

#define  ROBOT_MOTOR_SIZE              6
#define  MOTORINIT_TIME                1

namespace utilities{

namespace memory {

    template<typename T>
    class SHM
    {
        private:
        uint16_t SHM_id;
        uint16_t SHM_key;
        uint16_t SHM_size;
        uint16_t size_;

        public :    

        SHM(uint16_t key, uint16_t size);
        SHM() = delete;

        void SHM_GETID();
        int SHM_CREATE();
        int SHM_WRITE(T* data);
        int SHM_READ(T* smeomry);
        void SHM_FREE();

    };

    template<typename T>
    SHM<T>::SHM(uint16_t key, uint16_t size) : SHM_key(key), SHM_size(sizeof(T)*size), size_(size)
    {

        };

    template<typename T>
    void SHM<T>::SHM_GETID(){

        
        if((SHM_id = shmget((key_t)SHM_key, 0, 0666)) == -1)
        {
        perror("SHM_GETID : Failed to get SHM_ID");
        }
        printf("Success to get SHM_ID\n");

    }


    template<typename T>
    int SHM<T>::SHM_CREATE(){
        printf("SHM_KEY %d\n", SHM_key);
        printf("SHM_size %d\n", SHM_size);
    

        if((SHM_id = shmget((key_t)SHM_key, SHM_size, IPC_CREAT| IPC_EXCL | 0666)) == -1) {
            printf("SHM_CREATE : SHM already exist.\n");
            SHM_id = shmget((key_t)SHM_key, SHM_size, IPC_CREAT| 0666);
        
        if(SHM_id == -1)
        {
            printf("SHM_CREATE : SHM create fail");
            return 1;
        }
        printf("Success to Get SHM%d",SHM_key);
    }
    
    return 0;

    }

    template<typename T>
    int SHM<T>::SHM_WRITE(T* data)
    {
        void *SHMaddr = shmat(SHM_id,nullptr,0);

        if(SHMaddr == (void *)-1) 
        {
            printf("SHM_WRITE : Shmat failed error condition 1");
            return 1;
        }
        memcpy(SHMaddr, data, SHM_size);
        // if(shmdt(SHMaddr) == -1) 
        // {
        //     printf("SHM_WRITE : Shmdt failed error condition 2");
        //     return 1;
        // }
    return 0;

    }


    template<typename T>
    int SHM<T>::SHM_READ(T* smemory)
    {
        void *SHMaddr = shmat(SHM_id, nullptr,0);
        
        if(SHMaddr == (void *)-1)
        {
            printf("SHM_READ : Shmat failed error condition 1");
            return 1;
        }
        
        memcpy(smemory, SHMaddr, SHM_size);
        
        // if(shmdt(SHMaddr) == -1)
        // {
        //     printf("SHM_READ : Shmdt failed error condition 2");
        //     return 1;
        // }
    return 0;

    }
    
    template<typename T>
    void SHM<T>::SHM_FREE()
    {   
        SHM_GETID();
        if (SHM_id < 0) {
        std::cout << "SHM_FREE: Invalid SHM_id, cannot free memory." << std::endl;
        return;
        }
        if(shmctl(SHM_id, IPC_RMID, nullptr) == -1) 
        {
            printf("SHM_FREE : Shmctl failed");
            return ;
        }
    
    std::cout << "Shared memory (ID: " << SHM_id << ") successfully freed." << std::endl;

        return ;
    }


}


}






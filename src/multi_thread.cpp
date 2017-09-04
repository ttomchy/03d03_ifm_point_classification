#include <iostream>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include <unistd.h>




std::deque<int> data;
std::mutex m_mux;//全局互斥锁
int num=0;
void product_thread(){


    while(true){
        std::unique_lock<std::mutex> lck(m_mux);
        num = ++num % 1000;
        data.push_front(num);
        printf("product %d\n", num);
        lck.unlock();
        sleep(2);
    }
}
void consume_thread(){


    while (true){

        std::unique_lock<std::mutex> lck(m_mux);
        if(data.empty()) {
            lck.unlock();
            continue;
        }

        printf("consume %d\n", data.back());
        data.pop_back();
        lck.unlock();
        sleep(5);
    }
}

int main() {

    std::thread thread_product(&product_thread);
    std::cout<<"Now it runs after the thread_product function !"<<std::endl;

    std::thread thread (&consume_thread);
    std::cout<<"Now it runs after the thread function !"<<std::endl;

    thread_product.join();
    std::cout<<"Now it runs after the  thread_product function !"<<std::endl;

    thread.join();
    std::cout<<"Now it runs after the thread.join function !"<<std::endl;
    return  0;
}
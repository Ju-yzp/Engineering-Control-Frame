//multi thread 
#include <cstddef>
#include<queue>
#include<mutex>

template <typename T>
class SafeQueue{
    public:
    SafeQueue()=default;
    SafeQueue(SafeQueue &other)=delete;
    ~SafeQueue(){}

    bool empty(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    std::size_t size(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    void enqueue(T & t){
        std::unique_lock<std::mutex> lock(m_mutex);
        m_queue.emplace(t);
    }

    bool dequeue(T &t){
        std::unique_lock<std::mutex> lock(m_mutex);

        if(m_queue.empty()){
            return false;
        }
        t = std::move(m_queue.front());
        m_queue.pop();
        return true;
    }
    private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
};
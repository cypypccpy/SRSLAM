#include <srslam/backend.h>
#include <srslam/frontend.h>
class Stereo
{   
    public:  
        Stereo() {};
        
        /// 初始化，返回是否成功
        bool Init();

        ~Stereo(){}

    private:
        std::shared_ptr<Frontend> frontend_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<map> map_ = nullptr;
};  

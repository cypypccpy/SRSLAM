#include <srslam/backend.h>
#include <srslam/frontend.h>
struct CameraParam {
    double m_xi;

    double m_fx;
    double m_fy;
    double m_cx;
    double m_cy;

    double m_k1;
    double m_k2;
    double m_p1;
    double m_p2;

    double m_gamma1;
    double m_gamma2;
    double m_u0;
    double m_v0;
};

class Stereo
{   
    public:  
        Stereo() {};
        
        /// 初始化，返回是否成功
        bool Init();

        CameraParam readFromYamlFile(const std::string& filename);

        ~Stereo(){}

    private:
        std::shared_ptr<Frontend> frontend_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<map> map_ = nullptr;
};  

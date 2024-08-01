#pragma once

#include "camera/camera_geometry_base.h"
#include "system/base/point.h"

namespace YL_SLAM {

/**
 * @brief 帧类
 * @details 帧类包含了帧的基本信息，如时间戳、相机、图像金字塔及观测到的特征等
 */
class Frame : public NonCopyable {
public:
    using sPtr      = std::shared_ptr<Frame>;
    using sConstPtr = std::shared_ptr<const Frame>;

    /**
     * @brief 构造函数
     * @param timestamp 帧时间戳（ns）
     * @param camera 帧所属的相机指针
     * @param T_bc 帧所属相机坐标系到body坐标系（通常是IMU）的变换
     * @param image 帧原始图像
     * @param pyr_levels 图像金字塔层数
     */
    Frame(int64_t timestamp, const CameraGeometryBase::sPtr &camera, const SE3f &T_bc, cv::Mat image,
          size_t pyr_levels);

    /**
     * @brief 默认析构函数
     */
    ~Frame() = default;

    /**
     * @brief 获取帧时间戳（ns）
     * @return 帧时间戳
     */
    [[nodiscard]] int64_t timestamp() const;

    /**
     * @brief 获取帧id
     * @return 帧id
     */
    [[nodiscard]] long id() const;

    /**
     * @brief 获取帧所属的帧束id
     * @return 帧束id
     * @warning 如果返回-1，表示帧还没有被加入到帧束中
     */
    [[nodiscard]] long bundleId() const;

    /**
     * @brief 设置帧所属的帧束id
     * @param bundle_id 帧束id
     */
    void setBundleId(long bundle_id);

    /**
     * @brief 获取帧所属的相机指针
     * @return 帧所属的相机指针
     */
    [[nodiscard]] const CameraGeometryBase::sConstPtr &camera() const;

    /**
     * @brief 获取世界坐标系到帧坐标系的变换
     * @return 世界坐标系到帧坐标系的变换
     * @warning 帧创建时该值属于未设置状态，需要设置后才能使用
     */
    [[nodiscard]] const SE3f &Twf() const;

    /**
     * @brief 设置世界坐标系到帧坐标系的变换
     * @param T_wf 世界坐标系到帧坐标系的变换
     */
    void setTwf(const SE3f &T_wf);

    /**
     * @brief 获取帧所属相机坐标系到body坐标系（通常是IMU）的变换
     * @return 帧所属相机坐标系到body坐标系（通常是IMU）的变换
     */
    [[nodiscard]] const SE3f &Tbc() const;

    /**
     * @brief 设置帧所属相机坐标系到body坐标系（通常是IMU）的变换
     * @param T_bc 帧所属相机坐标系到body坐标系（通常是IMU）的变换
     */
    void setTbc(const SE3f &T_bc);

    /**
     * @brief 获取帧图像金字塔
     * @return 帧图像金字塔
     */
    [[nodiscard]] const ImagePyr &imagePyr() const;

    /**
     * @brief 获取帧原始图像
     * @return 帧原始图像
     */
    [[nodiscard]] const cv::Mat &rawImage() const;

    /**
     * @brief 添加该帧观测到的特征集合
     * @param kps 观测到的特征的二维像素坐标集合
     * @param f 观测到的特征的单位方向向量集合
     * @param points 观测到的特征的三维点集合
     */
    void addObservations(const Keypoints &kps, const Bearings &f, const std::vector<Point::sConstPtr> &points);

    /**
     * @brief 移除指定索引下该帧观测到的特征
     * @details 移除操作为直接将该索引的特征的三维点指针置空
     * @param idx 指定索引
     * @warning 该方法主要用于移除粗差观测
     */
    void removeObservation(size_t idx);

    /**
     * @brief 获取该帧观测到的特征数量
     * @return 该帧观测到的特征数量
     */
    [[nodiscard]] size_t numObservations() const;

    /**
     * @brief 获取指定索引下该帧观测到的特征的二维像素坐标
     * @param idx 指定索引
     * @return 指定索引下该帧观测到的特征的二维像素坐标
     */
    [[nodiscard]] Eigen::Ref<const Keypoint> obsKeypoint(size_t idx) const;

    /**
     * @brief 获取指定索引下该帧观测到的特征的单位方向向量
     * @param idx 指定索引
     * @return 指定索引下该帧观测到的特征的单位方向向量
     */
    [[nodiscard]] Eigen::Ref<const Bearing> obsBearing(size_t idx) const;

    /**
     * @brief 获取指定索引下该帧观测到的特征的三维点常量指针
     * @param idx 指定索引
     * @return 指定索引下该帧观测到的特征的三维点常量指针
     * @warning 可能返回空指针（该三维点被边缘化或为粗差被移除）
     */
    [[nodiscard]] Point::sConstPtr obsPoint(size_t idx) const;

    /**
     * @brief 添加由该帧生成的种子点集合
     * @param kps 由该帧生成的种子点的二维像素坐标集合
     * @param f 由该帧生成的种子点的单位方向向量集合
     * @param points 由该帧生成的种子点的三维点集合
     * @param seed_states 由该帧生成的种子点的状态集合
     */
    void addSeeds(const Keypoints &kps, const Bearings &f, const std::vector<Point::sPtr> &points,
                  const SeedStates &seed_states);

    /**
     * @brief 移除指定索引下由该帧生成的种子点
     * @details 移除操作为直接将该索引的种子点的三维点指针置空
     * @param idx 指定索引
     * @note 该方法主要用于移除粗差种子点
     */
    void removeSeed(size_t idx);

    /**
     * @brief 获取由该帧生成的种子点数量
     * @return 由该帧生成的种子点数量
     */
    [[nodiscard]] size_t numSeeds() const;

    /**
     * @brief 获取指定索引下由该帧生成的种子点的二维像素坐标
     * @param idx 指定索引
     * @return 指定索引下由该帧生成的种子点的二维像素坐标
     */
    [[nodiscard]] Eigen::Ref<const Keypoint> seedKeypoint(size_t idx) const;

    /**
     * @brief 获取指定索引下由该帧生成的种子点的单位方向向量
     * @param idx 指定索引
     * @return 指定索引下由该帧生成的种子点的单位方向向量
     */
    [[nodiscard]] Eigen::Ref<const Bearing> seedBearing(size_t idx) const;

    /**
     * @brief 获取指定索引下由该帧生成的种子点的三维点常量指针
     * @param idx 指定索引
     * @return 指定索引下由该帧生成的种子点的三维点常量指针
     * @warning 可能返回空指针（该三维点为粗差被移除）
     */
    [[nodiscard]] Point::sConstPtr seedPoint(size_t idx) const;

    /**
     * @brief 获取指定索引下由该帧生成的种子点的三维点指针
     * @param idx 指定索引
     * @return 指定索引下由该帧生成的种子点的三维点指针
     * @warning 可能返回空指针（该三维点为粗差被移除）
     */
    [[nodiscard]] Point::sPtr &mutableSeedPoint(size_t idx);

    /**
     * @brief 获取指定索引下由该帧生成的种子点的状态
     * @param idx 指定索引
     * @return 指定索引下由该帧生成的种子点的状态
     */
    [[nodiscard]] Eigen::Ref<const SeedState> seedState(size_t idx) const;

    /**
     * @brief 获取指定索引下由该帧生成的种子点的状态（可修改）
     * @param idx 指定索引
     * @return 指定索引下由该帧生成的种子点的状态（可修改）
     */
    [[nodiscard]] Eigen::Ref<SeedState> mutableSeedState(size_t idx);

private:
    /**
     * @brief 创建图像金字塔
     * @param image 输入图像
     * @param image_pyr 输出图像金字塔
     * @param pyr_levels 金字塔层数，包括最底层
     * @warning 输入图像的类型需要是CV_8UC1或CV_8UC3
     */
    static void createImagePyramid(const cv::Mat &image, ImagePyr &image_pyr, size_t pyr_levels);

    // 帧信息
    int64_t timestamp_;                    ///< 时间戳（ns）
    long id_;                              ///< 帧id（历史唯一）
    long bundle_id_{-1};                   ///< 帧束id（用于多目，历史唯一）
    CameraGeometryBase::sConstPtr camera_; ///< 帧所属的相机
    SE3f T_wf_;                            ///< 世界坐标系到帧坐标系的变换
    SE3f T_bc_;                            ///< 帧所属相机坐标系到body坐标系（通常是IMU）的变换
    ImagePyr image_pyr_;                   ///< 图像金字塔
    cv::Mat raw_image_;                    ///< 原始图像（可能是彩色图像）

    // 特征信息
    Keypoints obs_kp_vec_;                        ///< 该帧观测到特征的二维像素坐标
    Bearings obs_f_vec_;                          ///< 该帧观测到特征的单位方向向量
    std::vector<Point::wConstPtr> obs_point_vec_; ///< 该帧观测到特征的三维点指针
    Keypoints seed_kp_vec_;                       ///< 由该帧生成的种子点的二维像素坐标
    Bearings seed_f_vec_;                         ///< 由该帧生成的种子点的单位方向向量
    std::vector<Point::sPtr> seed_point_vec_;     ///< 由该帧生成的种子点的三维点指针
    SeedStates seed_state_vec_; ///< 由该帧生成的种子点的状态信息，顺序：逆深度，逆深度方差，光度参数a和b
};

} // namespace YL_SLAM

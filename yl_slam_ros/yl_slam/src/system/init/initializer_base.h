#pragma once

#include "common/non_copyable.h"
#include "common/sensor/imu.h"
#include "common/yaml/yaml_serialization.h"
#include "system/base/frame_bundle.h"

namespace YL_SLAM {

/**
 * @brief 初始化器基类
 * @details 该类为初始化器的基础类，所有初始化器都通过YAML配置文件实现动态加载
 */
class InitializerBase : public NonCopyable {
public:
    using uPtr = std::unique_ptr<InitializerBase>;

    /**
     * @brief 默认构造函数
     */
    InitializerBase() = default;

    /**
     * @brief 默认析构函数
     */
    virtual ~InitializerBase() = default;

    /**
     * @brief 从YAML节点中加载初始化器
     * @param config YAML节点
     * @param g_w 世界坐标系下的重力向量
     * @return 所加载的初始化器指针
     */
    static uPtr loadFromYaml(const YAML::Node &config, const Vec3f &g_w);

    /**
     * @brief 将初始化器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 进行初始化接口
     * @param bundle 新的帧束
     * @param imus 新的IMU数据
     * @return 是否初始化成功
     */
    virtual bool initialize(const FrameBundle::sPtr &bundle, const Imus &imus) = 0;

    /**
     * @brief 重置初始化器接口
     */
    virtual void reset() = 0;

    /**
     * @brief 获取是否已经初始化
     * @return 是否已经初始化
     */
    [[nodiscard]] bool isInitialized() const;

    /**
     * @brief 打印初始化器参数接口
     * @param out 输出流实例
     */
    virtual void print(std::ostream &out) const = 0;

    /**
     * @brief 获取初始化器类型
     * @return 初始化器类型
     */
    [[nodiscard]] virtual std::string type() const = 0;

    /**
     * @brief 获取初始化器参数
     * @return 初始化器参数
     */
    [[nodiscard]] virtual VecXf parameters() const = 0;

protected:
    bool initialized_{false}; ///< 初始化标志位
};

} // namespace YL_SLAM

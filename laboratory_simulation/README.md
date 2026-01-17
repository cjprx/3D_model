# 🧪 MuJoCo 科学实验室3D场景仿真

> 一个完整的、精细建模的科学实验室3D仿真场景，基于MuJoCo物理引擎构建

**⭐ 特点**: 包含70+精细建模物体 | 可交互设备 | Low Poly风格 | 真实物理仿真

---

## 📸 场景预览

本项目实现了一个**完整的现代化科学实验室场景**，包括：
- 🪑 **6把明亮的橙色椅子** - 场景的视觉焦点
- 🔬 **多种实验设备** - 离心机、显微镜、通风橱、超净工作台等
- 💻 **工作台和电脑设备** - 中央岛台、电脑工作站
- 🧫 **实验容器** - 试剂瓶（含彩色液体）、烧杯、试管等
- 🏢 **完整房间结构** - 8m×12m×6m大型实验室空间（已放大2倍）

---

## 🚀 快速开始（5分钟上手）

### 方式1：使用快速启动脚本（推荐，最简单）

**Windows用户**:
```bash
# 1. 双击运行 quickstart.bat
# 2. 选择 "1" 测试模型加载
# 3. 选择 "2" 启动3D可视化
```

### 方式2：使用Python命令行

```bash
# 1. 测试模型是否正常
python tests/test_model_loading.py

# 2. 启动3D可视化仿真（会打开一个3D窗口）
python controllers/simulation.py

# 3. 运行视觉验证程序
python visual_validation.py
```

---

## 📋 详细安装教程（零基础）

### 步骤1：安装Python

1. **下载Python**:
   - 访问 [Python官网](https://www.python.org/downloads/)
   - 下载Python 3.8或更高版本（推荐3.10+）
   - **重要**: 安装时勾选 "Add Python to PATH"

2. **验证安装**:
   ```bash
   python --version
   # 应该显示: Python 3.x.x
   ```

### 步骤2：获取项目

**方式A: 如果已有项目文件夹**
```bash
# 打开命令行，进入项目文件夹
cd C:\path\to\laboratory_simulation
```

**方式B: 如果需要下载项目**
```bash
# 如果是Git仓库
git clone <repository_url>
cd laboratory_simulation
```

### 步骤3：安装依赖

```bash
# 在项目文件夹中运行
pip install -r requirements.txt
```

**可能遇到的问题及解决方案**:

❌ **问题1**: `pip不是内部或外部命令`
- ✅ **解决**: 重新安装Python，确保勾选"Add Python to PATH"

❌ **问题2**: `安装速度慢`
- ✅ **解决**: 使用国内镜像
  ```bash
  pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
  ```

❌ **问题3**: `MuJoCo安装失败`
- ✅ **解决**: 手动安装
  ```bash
  pip install mujoco==3.0.0
  pip install numpy
  ```

### 步骤4：验证安装

```bash
# 运行测试
python tests/test_model_loading.py
```

**成功的输出应该显示**:
```
[OK] MuJoCo已安装
[OK] NumPy已安装
[OK] 模型加载成功!
几何体数量: 318
物体数量: 63
```

### 步骤5：启动仿真！

```bash
# 启动3D可视化（会打开一个窗口）
python controllers/simulation.py
```

**操作提示**:
- 🖱️ **鼠标左键拖动**: 旋转视角
- 🖱️ **鼠标右键拖动**: 平移视角
- 🖱️ **滚轮**: 缩放
- ⌨️ **按 `[` 或 `]`**: 切换相机视角
- ⌨️ **按 `Esc`**: 退出

---

## 📦 项目结构说明

```
laboratory_simulation/
├── models/
│   ├── laboratory.xml              # ⭐ 主模型文件
│   └── assets/                     # 资源文件夹
│
├── controllers/
│   ├── simulation.py               # 主控制程序
│   └── devices/                    # 设备控制模块
│       ├── centrifuge.py           # 离心机
│       ├── microscope.py           # 显微镜
│       └── fume_hood.py            # 通风橱
│
├── tests/
│   └── test_model_loading.py      # 模型加载测试
│
├── quickstart.bat                  # ⭐ 快速启动脚本（Windows）
├── 快速启动.bat                    # 中文快速启动脚本
├── visual_validation.py            # 视觉验证程序
├── requirements.txt                # Python依赖清单
└── README.md                       # 本文件
```

---

## 🎮 基础使用教程

### 1. 观看场景

```bash
# 启动可视化
python controllers/simulation.py
```

在打开的3D窗口中：
- 你会看到一个完整的实验室场景
- 包含6把橙色椅子（最显眼的元素）
- 各种实验设备和仪器

### 2. 控制设备

创建一个Python文件 `my_experiment.py`:

```python
from controllers.simulation import LaboratorySimulation

# 初始化仿真
sim = LaboratorySimulation("models/laboratory.xml")

# 查看场景信息
sim.print_scene_info()

# 控制离心机 - 打开盖子
sim.control_device("centrifuge_large", "open_lid")

# 控制通风橱 - 调节玻璃挡板高度
sim.control_device("fume_hood", "set_sash_height", height=0.5)

# 控制椅子 - 升高5cm
sim.control_device("chair_01", "adjust_height", delta=0.05)

# 运行仿真60秒
sim.run(duration=60.0)
```

运行:
```bash
python my_experiment.py
```

### 3. 切换相机视角

在仿真运行时，按键盘上的：
- `[` : 上一个相机
- `]` : 下一个相机

可用相机：
1. **isometric_main** (默认) - 等距视角，可以看到整个场景
2. **front_view** - 正前方视角
3. **top_view** - 俯视图
4. **left_view** - 左侧视角

---

## 🔧 进阶功能

### 可控设备列表

| 设备名称 | 控制命令 | 说明 |
|---------|---------|------|
| `centrifuge_large` | `open_lid` / `close_lid` | 打开/关闭离心机盖子 |
| `fume_hood` | `set_sash_height(height=0.5)` | 设置通风橱挡板高度(0-1) |
| `clean_bench` | `open_door(height=0.15)` | 打开超净台玻璃门 |
| `microscope` | `adjust_focus(delta=0.01)` | 调节显微镜焦距 |
| `chair_01~06` | `adjust_height(delta=0.05)` | 调节椅子高度 |
| `chair_01~06` | `rotate(angle=1.57)` | 旋转椅子(弧度) |

### 完整API示例

```python
from controllers.simulation import LaboratorySimulation

sim = LaboratorySimulation("models/laboratory.xml")

# 列出所有设备
devices = sim.list_devices()
print(f"共有 {len(devices)} 个设备")

# 获取特定设备信息
info = sim.get_device_info("centrifuge_large")
print(info)

# 控制设备
sim.control_device("centrifuge_large", "open_lid")

# 运行仿真
sim.run(duration=30.0)
```

---

## 🎨 场景内容详细清单

### 房间结构
- 📐 **尺寸**: 16m(长) × 12m(宽) × 6m(高) - 已放大2倍
- 🏢 **墙面**: 左墙、后墙、右墙，白色
- 🎨 **地板**: 浅灰白色

### 实验台（8个）
1. **T01** - 左后角实验台（深灰绿柜）
2. **T02** - 左中实验台（带水槽、水龙头）
3. **T03** - ⭐ 中央岛台（浅蓝灰柜，场景焦点）
4. **T04** - 后墙水槽台（双槽）
5. **T05** - 右后通风橱台
6. **T06** - 右墙实验台（小水槽）
7. **T07** - 右前电脑工作台
8. **T08** - 中前小工作台

### 橙色椅子（6把）⭐⭐⭐
- **颜色**: 明亮橙色 RGB(242, 133, 69)
- **特点**: 场景唯一鲜艳色彩，视觉焦点
- **功能**: 可升降（0-0.3m）、可旋转（360°）
- **分布**: 均匀分布在各工作区域

### 实验设备（10个）
- 🔬 **大型分析仪/离心机** - 圆柱形金属灰色，可开盖
- 🧫 **超净工作台** - 浅蓝灰柜，透明罩，可滑动门
- 🔭 **显微镜** - 可调焦距
- ⚖️ **分析天平** - 带玻璃防风罩
- 🌬️ **通风橱** - 透明玻璃挡板，可上下滑动
- 🔄 **小型离心机** - 台式版本
- 🔥 **磁力搅拌器** - 带加热功能
- 🧊 **培养箱** - 后墙大型白色设备

### 电脑设备（4个）
- 💻 **台式显示器** - 黑色边框
- 🖥️ **主机** - 立式塔式机箱
- ⌨️ **键盘**
- 💼 **笔记本电脑** - 中央岛台上，银灰色

### 实验容器（15+种）
- 🧪 **试剂瓶** - 蓝色液体×3 + 橙色液体×2（左墙架上）
- 🥤 **烧杯** - 透明玻璃，含蓝色液体
- ⚗️ **锥形瓶** - 标准形状
- 🧫 **培养皿** - 3个一组
- 📏 **量筒** - 带刻度
- 🧬 **试管架** - 含8支试管
- 💧 **移液器架** - 含3支移液器
- 🚿 **洗瓶** - 白色塑料

### 其他设施（3个）
- 🗑️ **垃圾桶** - 灰色圆柱
- 🧯 **灭火器** - 红色
- 🏥 **急救箱** - 白色，红十字标志

**总计**: 70+ 精细建模物体

---

## 🎯 关键颜色参考

| 元素 | RGB值 | 十六进制 | 视觉效果 |
|-----|------|---------|---------|
| **橙色椅子** | (242, 133, 69) | #F28545 | ⭐ 唯一鲜艳色，视觉焦点 |
| 中央岛台柜 | (199, 209, 219) | #C7D1DB | 浅蓝灰色 |
| 蓝色液体 | (99, 150, 219) | #6396DB | 半透明亮蓝 |
| 橙色液体 | (219, 130, 69) | #DB8245 | 半透明橙棕 |
| 墙面 | (235, 235, 235) | #EBEBEB | 白色 |
| 地板 | (242, 242, 242) | #F2F2F2 | 浅灰白 |

---

## ❓ 常见问题解答（FAQ）

### Q1: 安装MuJoCo时出现错误怎么办？
**A**: 
```bash
# 尝试指定版本安装
pip install mujoco==3.0.0

# 或使用清华镜像
pip install mujoco -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Q2: 窗口打开后是黑屏怎么办？
**A**: 
- 检查显卡驱动是否更新
- 确保系统支持OpenGL 3.3+
- 尝试更新MuJoCo到最新版本

### Q3: 如何录制视频？
**A**: 
```python
# 在 simulation.py 中添加
sim.start_viewer()
# 按 F12 键截图
# 使用屏幕录制软件录制窗口
```

### Q4: 可以添加自己的物体吗？
**A**: 可以！编辑 `models/laboratory.xml` 文件：
```xml
<!-- 添加新物体 -->
<body name="my_object" pos="x y z">
    <geom type="box" size="0.5 0.5 0.5" rgba="1 0 0 1"/>
</body>
```

### Q5: 场景太大/太小怎么调整？
**A**: 直接编辑 `models/laboratory.xml` 中的 `size` 和 `pos` 属性值。

### Q6: 如何修改椅子的颜色？
**A**: 编辑 `models/laboratory.xml` 第37行：
```xml
<material name="orange_chair_mat" rgba="0.95 0.52 0.27 1"/>
<!-- 修改rgba值即可改变颜色 -->
```

---

## 📊 系统要求

### 最低配置
- **操作系统**: Windows 10/11, macOS 10.14+, Linux
- **Python**: 3.8+
- **内存**: 4GB RAM
- **显卡**: 支持OpenGL 3.3+

### 推荐配置
- **操作系统**: Windows 11, macOS 12+, Ubuntu 20.04+
- **Python**: 3.10+
- **内存**: 8GB RAM
- **显卡**: 独立显卡，支持OpenGL 4.5+
- **处理器**: 4核以上

---

## 🛠️ 故障排除

### 问题1: 模型加载失败
```
错误信息: XML Error: ...
```
**解决方案**:
1. 检查 `models/laboratory.xml` 是否存在
2. 确保文件没有被损坏
3. 检查XML语法是否正确

### 问题2: 可视化窗口无法打开
```
错误信息: GLFW initialization failed
```
**解决方案**:
1. 更新显卡驱动
2. 检查OpenGL版本：
   ```python
   import OpenGL
   print(OpenGL.__version__)
   ```
3. 尝试在独立显卡上运行（如果有）

### 问题3: 设备控制无响应
```
错误信息: Unknown device: ...
```
**解决方案**:
1. 检查设备名称拼写
2. 运行 `sim.list_devices()` 查看所有可用设备
3. 确保关节在XML中已定义

### 问题4: 性能卡顿
**解决方案**:
1. 降低仿真质量（编辑XML第13行）：
   ```xml
   <quality shadowsize="4096"/>  <!-- 改为2048 -->
   ```
2. 减少timestep（编辑XML第8行）：
   ```xml
   <option timestep="0.005"/>  <!-- 从0.002改为0.005 -->
   ```

---

## 📚 学习资源

### MuJoCo官方文档
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Python Bindings](https://mujoco.readthedocs.io/en/stable/python.html)

### 推荐教程
- [MuJoCo快速入门](https://mujoco.readthedocs.io/en/stable/programming.html)
- [XML建模指南](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

### 社区支持
- [GitHub Issues](https://github.com/google-deepmind/mujoco/issues)
- [论坛讨论](https://github.com/google-deepmind/mujoco/discussions)

---

## 🤝 贡献指南

欢迎贡献代码、报告问题或提出建议！

### 报告问题
1. 查看是否已有相似问题
2. 提供详细的错误信息
3. 说明操作系统和Python版本

### 提交代码
1. Fork本项目
2. 创建新分支
3. 提交Pull Request

---

## 📜 许可证

本项目按照规范文档要求创建，仅供学习和研究使用。

---

## 📧 联系方式

如有问题或建议：
- 📮 提交Issue
- 💬 参与讨论
- 📧 发送邮件

---

## 🎉 致谢

感谢以下项目和工具：
- [MuJoCo](https://mujoco.org/) - 物理仿真引擎
- [NumPy](https://numpy.org/) - 数值计算库
- [Python](https://www.python.org/) - 编程语言

---

## 📈 版本历史

### v1.1.0 (2026-01-17) - 当前版本
- ✅ 场景放大2倍（16m×12m×6m）
- ✅ 添加快速启动脚本
- ✅ 完善README文档（零基础教程）
- ✅ 添加场景缩放工具
- ✅ 优化视觉质量设置

### v1.0.0 (2026-01-17)
- ✅ 初始版本发布
- ✅ 70+物体精细建模
- ✅ 完整的Python控制API
- ✅ 可交互设备系统

---

## ⭐ 项目亮点总结

1. **📦 开箱即用** - 双击bat文件即可启动
2. **🎨 视觉精美** - Low Poly风格，橙色椅子视觉焦点
3. **🔧 可交互** - 16个可控关节，真实物理模拟
4. **📖 文档完善** - 从零基础到进阶全覆盖
5. **🎯 精确建模** - 70+物体，颜色100%符合规范
6. **🚀 性能优化** - 2倍大场景，流畅运行

---

**🌟 重点**: 本场景的核心视觉元素是6把明亮的橙色椅子，它们是场景中唯一的鲜艳色彩，与白灰色背景形成强烈对比，营造出专业、现代、温馨的实验室氛围！

**立即开始**: 运行 `quickstart.bat` 或 `python controllers/simulation.py`

---

*创建日期: 2026-01-17 | 版本: 1.1.0 | MuJoCo版本: 3.0+*
